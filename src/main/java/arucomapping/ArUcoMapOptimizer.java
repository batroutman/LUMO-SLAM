package arucomapping;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

import org.ddogleg.optimization.lm.ConfigLevenbergMarquardt;
import org.ejml.data.DMatrixRMaj;

import Jama.Matrix;
import boofcv.abst.geo.bundle.BundleAdjustment;
import boofcv.abst.geo.bundle.PruneStructureFromSceneMetric;
import boofcv.abst.geo.bundle.ScaleSceneStructure;
import boofcv.abst.geo.bundle.SceneObservations;
import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.alg.geo.bundle.cameras.BundlePinhole;
import boofcv.factory.geo.ConfigBundleAdjustment;
import boofcv.factory.geo.FactoryMultiView;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;
import lumoslam.Keyframe;
import lumoslam.Photogrammetry;
import runtimevars.Parameters;
import toolbox.Utils;
import types.Correspondence2D2D;
import types.Dbl;
import types.Point3D;

public class ArUcoMapOptimizer {

	public static double PRUNING_THRESHOLD = 50;
	public static double PRUNING_INLIER_THRESHOLD = 0.80;
	public static double ITERATIVE_PRUNING_INLIER_THRESHOLD = 0.95;

	protected ArUcoMap markerMap = null;
	protected List<Thread> threads = new ArrayList<Thread>();
	public ThreadGroup threadGroup = new ThreadGroup("Marker Map Optimizer Thread Group");

	private ArUcoMapOptimizer() {
		this.initThreads();
	}

	public ArUcoMapOptimizer(ArUcoMap markerMap) {
		this.markerMap = markerMap;
		this.initThreads();
	}

	protected void initThreads() {

		Thread processThread = new Thread(threadGroup, "processThread") {
			@Override
			public void run() {

				if (!Parameters.<Boolean>get("usingMarkers")) {
					return;
				}

				Utils.pl("Starting ArUcoMapOptimization thread...");

				long sleepTime = 5;
				boolean keepGoing = true;
				while (keepGoing) {
					try {

						// triangulate untriangulated points
						triangulatePoints();

						// bundle adjustment?
						bundleAdjustment(10);

						// estimate scale
						estimateScale();

						// localize
						updateModelTransformation();

						Thread.sleep(sleepTime);
					} catch (InterruptedException e) {
						Utils.pl("markerThread interrupted");
						keepGoing = false;
					} catch (Exception e) {
						Utils.pl("Exception in markerThread loop:");
						e.printStackTrace();
					}

				}

			}
		};

		this.threads.add(processThread);

	}

	public void start() throws InterruptedException {

		for (Thread thread : this.threads) {
			thread.start();
		}

		for (Thread thread : this.threads) {
			thread.join();
		}

	}

	public void triangulatePoints() {

		double REPROJ_ERR_THRESH = 4;

		// get map points that are untriangulated
		List<ArUcoMapPoint> untriangulatedMapPoints = this.markerMap.getMapPoints().stream()
				.filter(mapPoint -> mapPoint.getTriangulatedPoint() == null).collect(Collectors.toList());

		// attempt to triangulate each point with widest baseline observations
		for (ArUcoMapPoint mapPoint : untriangulatedMapPoints) {

			List<Keyframe> observations = new ArrayList<Keyframe>(mapPoint.getObservations().keySet());
			if (observations.size() < 2) {
				continue;
			}

			// -- find widest baseline in observations
			double largestBaseline = 0;
			Keyframe primaryKeyframe = null;
			Keyframe secondaryKeyframe = null;

			for (int i = 0; i < observations.size() - 1; i++) {
				for (int j = i + 1; j < observations.size(); j++) {
					double baseline = observations.get(i).getPose().getDistanceFrom(observations.get(j).getPose());
					if (baseline > largestBaseline) {
						largestBaseline = baseline;
						primaryKeyframe = observations.get(i);
						secondaryKeyframe = observations.get(j);
					}
				}
			}

			// if largest baseline is big enough, triangulate the point
			if (largestBaseline > 2) {

				ArUcoKeypoint primaryKeypoint = mapPoint.getObservations().get(primaryKeyframe);
				ArUcoKeypoint secondaryKeypoint = mapPoint.getObservations().get(secondaryKeyframe);

				Correspondence2D2D wideC = new Correspondence2D2D();
				wideC.setX0(primaryKeypoint.locationX);
				wideC.setY0(primaryKeypoint.locationY);
				wideC.setX1(secondaryKeypoint.locationX);
				wideC.setY1(secondaryKeypoint.locationY);

				Dbl parallax = new Dbl(0);
				Dbl err0 = new Dbl(0);
				Dbl err1 = new Dbl(0);
				Dbl w0 = new Dbl(0);
				Dbl w1 = new Dbl(0);
				Matrix pointMatrix = Photogrammetry.triangulate(secondaryKeyframe.getPose().getHomogeneousMatrix(),
						primaryKeyframe.getPose().getHomogeneousMatrix(), wideC, parallax, err1, err0, w1, w0);

				// check that parallax is sufficient
				if (parallax.getValue() < 2) {
					continue;
				}

				// check cheirality
				if (w0.getValue() < 0 || w1.getValue() < 0) {
					continue;
				}

				// check reprojection errors
				if (err0.getValue() > REPROJ_ERR_THRESH || err1.getValue() > REPROJ_ERR_THRESH) {
					continue;
				}

				// otherwise, the triangulation should be good, update triangulated point
				mapPoint.setTriangulatedPoint(
						new Point3D(pointMatrix.get(0, 0), pointMatrix.get(1, 0), pointMatrix.get(2, 0)));

			}

		}

	}

	public void estimateScale() {

		// isolate list of triangulated points
		List<ArUcoMapPoint> triangulatedPoints = this.markerMap.getMapPoints().stream()
				.filter(mapPoint -> mapPoint.getTriangulatedPoint() != null).collect(Collectors.toList());

		if (triangulatedPoints.size() < 2) {
			return;
		}

		// generate pairs of points
		Collections.shuffle(triangulatedPoints);
		int selectionDepth = 3;
		HashMap<ArUcoMapPoint, List<ArUcoMapPoint>> pairings = new HashMap<ArUcoMapPoint, List<ArUcoMapPoint>>();
		for (int i = 0; i < triangulatedPoints.size() - selectionDepth; i++) {
			ArUcoMapPoint mapPoint = triangulatedPoints.get(i);
			List<ArUcoMapPoint> pairedPoints = new ArrayList<ArUcoMapPoint>();
			for (int j = i + 1; j < triangulatedPoints.size(); j++) {
				pairedPoints.add(triangulatedPoints.get(j));
			}
			pairings.put(mapPoint, pairedPoints);
		}

		// record ratio of triangulated distance to model distance in each pair
		List<Double> ratios = new ArrayList<Double>();
		for (ArUcoMapPoint mapPoint0 : pairings.keySet()) {
			for (ArUcoMapPoint mapPoint1 : pairings.get(mapPoint0)) {

				double modelDist = Utils.pointDistance(mapPoint0.getPoint(), mapPoint1.getPoint());
				double estimatedDist = Utils.pointDistance(mapPoint0.getTriangulatedPoint(),
						mapPoint1.getTriangulatedPoint());

				double ratio = estimatedDist / modelDist;
				ratios.add(ratio);

			}
		}

		// select median of scales?
		Dbl avg = new Dbl(0);
		Dbl stdDev = new Dbl(0);
		Dbl median = new Dbl(0);
		Utils.basicStats(ratios, avg, stdDev, median);

		this.markerMap.setScale(median.getValue());

	}

	public void updateModelTransformation() {

		// get traingulated map points
		List<ArUcoMapPoint> mapPoints = this.markerMap.getMapPoints().stream()
				.filter(mapPoint -> mapPoint.getTriangulatedPoint() != null).collect(Collectors.toList());

		// if not enough map points, skip
		if (mapPoints.size() < 6) {
			return;
		}

		// construct scaled model matrix
		double scale = this.markerMap.getScale();
		Matrix modelPoints = new Matrix(4, mapPoints.size(), 1);
		for (int i = 0; i < mapPoints.size(); i++) {
			Point3D point = mapPoints.get(i).getPoint();
			modelPoints.set(0, i, scale * point.getX());
			modelPoints.set(1, i, scale * point.getY());
			modelPoints.set(2, i, scale * point.getZ());
		}

		// construct non-scaled triangulated matrix
		Matrix triangulatedPoints = new Matrix(4, mapPoints.size(), 1);
		for (int i = 0; i < mapPoints.size(); i++) {
			Point3D point = mapPoints.get(i).getTriangulatedPoint();
			triangulatedPoints.set(0, i, point.getX());
			triangulatedPoints.set(1, i, point.getY());
			triangulatedPoints.set(2, i, point.getZ());
		}

		// get inverse of scaled model matrix
		Matrix modelInv = Utils.pseudoInverse(modelPoints);

		// transformation matrix = triangulated matrix * inverse of scaled model matrix
		Matrix transformationMatrix = triangulatedPoints.times(modelInv);

		// convert transformation matrix to Transformation object
		DMatrixRMaj rotation = new DMatrixRMaj(3, 3);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				rotation.set(i, j, transformationMatrix.get(i, j));
			}
		}
		Quaternion_F64 q64 = new Quaternion_F64();
		ConvertRotation3D_F64.matrixToQuaternion(rotation, q64);

		// update marker map transformation
		this.markerMap.getTransformation().setQw(q64.w);
		this.markerMap.getTransformation().setQx(q64.x);
		this.markerMap.getTransformation().setQy(q64.y);
		this.markerMap.getTransformation().setQz(q64.z);
		this.markerMap.getTransformation().setT(transformationMatrix.get(0, 3), transformationMatrix.get(1, 3),
				transformationMatrix.get(2, 3));

	}

	public void bundleAdjustment(int iterations) throws Exception {

		SceneStructureMetric scene = new SceneStructureMetric(false);
		SceneObservations sceneObservations = new SceneObservations();

		// extract data from map
		List<Keyframe> keyframes = new ArrayList<Keyframe>();
		HashMap<Keyframe, Integer> keyframeToIndex = new HashMap<Keyframe, Integer>();
		HashMap<ArUcoMapPoint, List<Keyframe>> mapPointToKeyframes = new HashMap<ArUcoMapPoint, List<Keyframe>>();
		List<ArUcoMapPoint> mapPoints = new ArrayList<ArUcoMapPoint>();

		// get all keyframes observing marker map
		int keyframeIndex = 0;
		for (ArUcoMapPoint mapPoint : this.markerMap.getMapPoints()) {

			if (mapPoint.getTriangulatedPoint() == null) {
				continue;
			}

			// add map point
			mapPoints.add(mapPoint);

			// add keyframes
			List<Keyframe> observedKeyframes = new ArrayList<Keyframe>(mapPoint.getObservations().keySet());
			mapPointToKeyframes.put(mapPoint, observedKeyframes);

			for (Keyframe keyframe : observedKeyframes) {
				if (keyframeToIndex.get(keyframe) == null) {
					keyframes.add(keyframe);
					keyframeToIndex.put(keyframe, keyframeIndex++);
				}
			}

		}

		if (keyframes.size() < 2) {
			return;
		}

		if (mapPoints.size() < 6) {
			return;
		}

		// ---- load data into BoofCV ---- //
		scene.initialize(keyframes.size(), keyframes.size(), mapPoints.size());
		sceneObservations.initialize(keyframes.size());

		// load camera poses into scene
		BundlePinhole camera = new BundlePinhole();
		camera.fx = Parameters.<Float>get("fx").doubleValue();
		camera.fy = Parameters.<Float>get("fy").doubleValue();
		camera.cx = Parameters.<Float>get("cx").doubleValue();
		camera.cy = Parameters.<Float>get("cy").doubleValue();
		camera.skew = Parameters.<Float>get("s").doubleValue();
		for (int i = 0; i < keyframes.size(); i++) {
			Se3_F64 worldToCameraGL = new Se3_F64();
			ConvertRotation3D_F64.quaternionToMatrix(keyframes.get(i).getPose().getQw(),
					keyframes.get(i).getPose().getQx(), keyframes.get(i).getPose().getQy(),
					keyframes.get(i).getPose().getQz(), worldToCameraGL.R);
			worldToCameraGL.T.x = keyframes.get(i).getPose().getTx();
			worldToCameraGL.T.y = keyframes.get(i).getPose().getTy();
			worldToCameraGL.T.z = keyframes.get(i).getPose().getTz();
			scene.setCamera(i, true, camera);
			scene.setView(i, true, worldToCameraGL);
			scene.connectViewToCamera(i, i);
		}

		// load points
		for (int i = 0; i < mapPoints.size(); i++) {
			float x = (float) mapPoints.get(i).getTriangulatedPoint().getX();
			float y = (float) mapPoints.get(i).getTriangulatedPoint().getY();
			float z = (float) mapPoints.get(i).getTriangulatedPoint().getZ();
			scene.setPoint(i, x, y, z);
			for (Keyframe keyframe : mapPointToKeyframes.get(mapPoints.get(i))) {
				scene.getPoints().get(i).views.add(keyframeToIndex.get(keyframe));
			}

		}

		// load observations
		for (int i = 0; i < mapPoints.size(); i++) {
			ArUcoMapPoint mp = mapPoints.get(i);
			for (Keyframe keyframe : mp.getObservations().keySet()) {
				int pointID = i;
				int viewID = keyframeToIndex.get(keyframe);
				double x = mp.getObservations().get(keyframe).locationX;
				double y = mp.getObservations().get(keyframe).locationY;
				sceneObservations.getView(viewID).add(pointID, (float) x, (float) y);
			}
		}

		// ---- prune and bundle adjustment ---- //
		// omit top x% of observations with highest reprojection error
		PruneStructureFromSceneMetric pruner = new PruneStructureFromSceneMetric(scene, sceneObservations);

		// perform BA
		boolean BAIncomplete = true;
		int maxAttempts = 3;
		for (int i = 0; i <= maxAttempts && BAIncomplete; i++) {
			try {
				this.bundleAdjustScene(scene, sceneObservations, iterations);
				BAIncomplete = false;
			} catch (Exception e) {
				if (i >= maxAttempts) {
					throw e;
				}
				Utils.pl("Marker BA failed, attempting to prune...");
				pruner.pruneObservationsByErrorRank(ITERATIVE_PRUNING_INLIER_THRESHOLD);
			}
		}

		// load points from scene back into input
		for (int i = 0; i < scene.getPoints().size(); i++) {
			mapPoints.get(i).getTriangulatedPoint().setX(scene.getPoints().get(i).getX());
			mapPoints.get(i).getTriangulatedPoint().setY(scene.getPoints().get(i).getY());
			mapPoints.get(i).getTriangulatedPoint().setZ(scene.getPoints().get(i).getZ());
		}

	}

	public void bundleAdjustScene(SceneStructureMetric scene, SceneObservations observations, int maxIterations)
			throws Exception {

		ConfigLevenbergMarquardt configLM = new ConfigLevenbergMarquardt();
		configLM.dampeningInitial = 1e-3;
		configLM.hessianScaling = true;

		ConfigBundleAdjustment configSBA = new ConfigBundleAdjustment();
		configSBA.configOptimizer = configLM;
		BundleAdjustment<SceneStructureMetric> bundleAdjustment = FactoryMultiView.bundleSparseMetric(configSBA);

		// debug
//		bundleAdjustment.setVerbose(System.out, 0);

		// Specifies convergence criteria
		bundleAdjustment.configure(1e-12, 1e-12, maxIterations);

		// Scaling each variable type so that it takes on a similar numerical
		// value. This aids in optimization
		// Not important for this problem but is for others
		ScaleSceneStructure bundleScale = new ScaleSceneStructure();
		bundleScale.applyScale(scene, observations);
		bundleAdjustment.setParameters(scene, observations);

		// Runs the solver. This will take a few minutes. 7 iterations takes
		// about 3 minutes on my computer
		long startTime = System.currentTimeMillis();
		double errorBefore = bundleAdjustment.getFitScore();
		Utils.pl("error before: " + errorBefore);
		if (!bundleAdjustment.optimize(scene)) {
			// throw new RuntimeException("Bundle adjustment failed?!?");
			Utils.pl("\n\n\n\n***************************  ERROR  ****************************");
			Utils.pl("NOTE: Bundle Adjustment failed!");
			Utils.pl("fit score: " + bundleAdjustment.getFitScore());
			bundleScale.undoScale(scene, observations);

			Utils.pl("****************************************************************\n\n\n\n");
			throw new Exception();
		}

		// Print out how much it improved the model
		System.out.println();
		System.out.printf("Error reduced by %.1f%%\n", (100.0 * (errorBefore / bundleAdjustment.getFitScore() - 1.0)));
		System.out.println("new error: " + bundleAdjustment.getFitScore());
		System.out.println((System.currentTimeMillis() - startTime) / 1000.0);

		// Return parameters to their original scaling. Can probably skip this
		// step.
		bundleScale.undoScale(scene, observations);

	}

}
