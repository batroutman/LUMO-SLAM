package lumoslam;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.ddogleg.optimization.lm.ConfigLevenbergMarquardt;
import org.opencv.core.KeyPoint;

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
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;
import loopclosure.LoopClosure;
import runtimevars.Parameters;
import toolbox.Timer;
import toolbox.Utils;
import types.Correspondence2D2D;
import types.Dbl;
import types.Point3D;
import types.Pose;

public class MapOptimizer {

	public static double PRUNING_THRESHOLD = 50;
	public static double PRUNING_INLIER_THRESHOLD = 0.80;
	public static double ITERATIVE_PRUNING_INLIER_THRESHOLD = 0.95;

	public static int totalObsvPruned = 0;
	public static int totalPointsPruned = 0;
	public static int totalToTriangulate = 0;

	public Object principalDescriptorLock = new Object();
	public Object pointPrunerLock = new Object();
	public Object baLock = new Object();

	protected Map map = null;
	protected MapOptimizer self = this;

	public ThreadGroup threadGroup = new ThreadGroup("Map Optimizer Thread Group");

	public MapOptimizer() {

	}

	public MapOptimizer(Map map) {
		this.map = map;
	}

	public Thread principleDescriptorThread = new Thread(threadGroup, "pricipleDescriptorThread") {
		@Override
		public void run() {

			long sleepTime = 16;
			boolean keepGoing = true;
			while (keepGoing) {
				try {
					synchronized (principalDescriptorLock) {
						updatePricipleObservations();
					}
					Thread.sleep(sleepTime);
				} catch (InterruptedException e) {
					Utils.pl("pricipleDescriptorThread interrupted");
					keepGoing = false;
				} catch (Exception e) {
					Utils.pl("Exception in pricipleDescriptorThread loop:");
					e.printStackTrace();
				}

			}

		}
	};

	public Thread pointPrunerThread = new Thread(threadGroup, "pointPrunerThread") {
		@Override
		public void run() {

			long sleepTime = 16;
			boolean keepGoing = true;
			while (keepGoing) {
				try {

					synchronized (pointPrunerLock) {
						prunePoints();

						// clean MO points out of table and repartition points
						cleanPartitionTable();
					}
					Thread.sleep(sleepTime);
				} catch (InterruptedException e) {
					Utils.pl("pointPrunerThread interrupted");
					keepGoing = false;
				} catch (Exception e) {
					Utils.pl("Exception in pointPrunerThread loop:");
					e.printStackTrace();
				}

			}

		}
	};

	public Thread BAThread = new Thread(threadGroup, "BAThread") {
		@Override
		public void run() {

			long sleepTime = 5;
			boolean keepGoing = true;
			while (keepGoing) {
				try {
					synchronized (baLock) {
						List<Keyframe> lastNKF = getLastNKeyframes(10, 0);
						pruneOutliers(lastNKF);
						windowedBundleAdjustment(lastNKF, 10);
					}
					Thread.sleep(sleepTime);
				} catch (InterruptedException e) {
					Utils.pl("BAThread interrupted");
					keepGoing = false;
				} catch (Exception e) {
					Utils.pl("Exception in BAThread loop:");
					e.printStackTrace();
				}

			}

		}
	};

	// perform long-form bundle adjustments for high-quality map refinement
	public Thread longBAThread = new Thread(threadGroup, "longBAThread") {
		@Override
		public void run() {

			int windowSize = 60;
			int followingDistance = 15;
			int startKF = 0;
			int endKF = startKF + windowSize;

			int numBAs = 0;

			long sleepTime = 100;
			boolean keepGoing = true;
			while (keepGoing) {
				try {
					synchronized (baLock) {
						if (endKF < self.map.getKeyframes().size() - followingDistance) {
							Timer t = new Timer();
							List<Keyframe> window = new ArrayList<Keyframe>(
									self.map.getKeyframes().subList(startKF, endKF + 1));
							pruneOutliers(window);
							boolean completed = self.fixedWindowBundleAdjustment(window, true, true, 20, true);
							startKF = startKF + windowSize / 2;
							endKF = startKF + windowSize;
							numBAs += completed ? 1 : 0;
							long baTime = t.stop();
							Utils.pl("Long-form BA completed (startKF: " + startKF + ", endKF: " + endKF + "): "
									+ baTime + "ms");
						}
					}
					Thread.sleep(sleepTime);
				} catch (InterruptedException e) {
					Utils.pl("longBAThread interrupted");
					Utils.pl("Number of long-form bundle adjustments: " + numBAs);
					keepGoing = false;
				} catch (Exception e) {
					Utils.pl("Exception in longBAThread loop:");
					e.printStackTrace();
				}

			}

		}
	};

	public Thread loopClosureThread = new Thread(threadGroup, "loopClosureThread") {
		@Override
		public void run() {

			LoopClosure loopClosure = new LoopClosure(map, self);

			long sleepTime = 5;
			boolean keepGoing = true;
			while (keepGoing) {
				try {
					loopClosure.checkLoopClosure();
					Thread.sleep(sleepTime);
				} catch (InterruptedException e) {
					Utils.pl("loopClosureThread interrupted");
					keepGoing = false;
				} catch (Exception e) {
					Utils.pl("Exception in loopClosureThread loop:");
					e.printStackTrace();
				}

			}

		}
	};

	public void start() throws InterruptedException {

		this.principleDescriptorThread.start();
		this.pointPrunerThread.start();
		this.BAThread.start();
		this.longBAThread.start();
		this.loopClosureThread.start();
		this.principleDescriptorThread.join();
		this.pointPrunerThread.join();
		this.BAThread.join();
		this.longBAThread.join();
		this.loopClosureThread.join();

	}

	public void updatePricipleObservations() {
		if (this.map.getMapPointsNeedingUpdatedPD().keySet().size() < 1) {
			return;
		}
		List<MapPoint> mps = new ArrayList<MapPoint>();
		synchronized (this) {
			mps.addAll(this.map.getMapPointsNeedingUpdatedPD().keySet());
		}
//		Utils.pl("Number of map points needing update to priciple descriptor: " + mps.size());
		for (MapPoint mp : mps) {
			if (mp.getMergedTo() == null) {
				mp.computePrincipleObservation();
			}
			this.map.getMapPointsNeedingUpdatedPD().remove(mp);
		}
	}

	// remove map points with associated objects from partition table and update
	// partitions
	public void cleanPartitionTable() {

		List<String> keys = new ArrayList<String>();
		synchronized (this.map) {
			keys.addAll(this.map.getMapPointsByPartition().keySet());
		}

		for (String key : keys) {
			synchronized (this.map) {
				List<MapPoint> partition = this.map.getMapPointsByPartition().get(key);
				List<MapPoint> partitionCopy = new ArrayList<MapPoint>(partition);
				for (int i = partitionCopy.size() - 1; i >= 0; i--) {
					MapPoint mp = partitionCopy.get(i);
					partition.remove(mp);
					if (mp.getAssociatedObject() != null || mp.getMergedTo() != null) {
						partitionCopy.remove(mp);
						continue;
					}
				}
				this.map.partitionMapPoints(partitionCopy);
			}
		}

		// repartition points
		for (String key : keys) {
			synchronized (this.map) {

				List<MapPoint> partition = this.map.getMapPointsByPartition().get(key);
				for (int i = partition.size() - 1; i >= 0; i--) {

					// if it is partitioned incorrectly, remove and re-partition
					MapPoint mp = partition.get(i);
					if (!key.equals(map.getPartitionKey(mp.getPoint()))) {
						partition.remove(i);
						this.map.partitionMapPoint(mp);
					}

				}

			}
		}

	}

	// scan through triangulated map points and test for quality
	public void prunePoints() {
		List<MapPoint> uncheckedMapPoints = new ArrayList<MapPoint>();
		synchronized (this.map) {
			uncheckedMapPoints.addAll(this.map.getUncheckedMapPoints());
		}

//		Utils.pl("uncheckedMapPoints.size(): " + uncheckedMapPoints.size());
		for (int i = 0; i < uncheckedMapPoints.size(); i++) {

			MapPoint mp = uncheckedMapPoints.get(i);

			// if moving object, ignore pruning
			if (mp.getAssociatedObject() != null) {
				synchronized (this.map) {
					this.map.getUncheckedMapPoints().remove(mp);
				}
				continue;
			}

			// if merged to another map point, prune
			if (mp.getMergedTo() != null) {
				this.disableMapPoint(mp);
				continue;
			}

			// get latest keyframe and number of keyframes
			long highestKFID = 0;
			HashMap<Keyframe, Boolean> keyframesUsed = new HashMap<Keyframe, Boolean>();
			for (int j = 0; j < mp.getObservations().size(); j++) {
				Observation o = mp.getObservations().get(j);
				Keyframe kf = o.getKeyframe();
				keyframesUsed.put(kf, true);
				if (kf.getID() > highestKFID) {
					highestKFID = kf.getID();
				}
			}
			int numKeyframes = keyframesUsed.keySet().size();

			// if not enough keyframes have passed since the triangulation of this map
			// point, skip
			if (highestKFID + 4 > this.map.getLatestKFID()) {
				continue;
			}

			// first case, not enough keyframes
			if (numKeyframes < 3) {
				// prune
				this.disableMapPoint(mp);
			} else if ((double) mp.getTimesTracked() / (mp.getTimesTracked() + mp.getTimesLost()) < 0.25) {
				// second case, poor tracking
				// prune
				this.disableMapPoint(mp);
			} else {
				// point is good, keep it
				synchronized (this.map) {
					this.map.getUncheckedMapPoints().remove(mp);
				}
			}

		}
	}

	// untriangulate and unpartition map point and destroy its observations
	public void disableMapPoint(MapPoint mp) {
		synchronized (this.map) {
			this.map.unPartitionMapPoint(mp);
			this.map.getAllPoints().remove(mp.getPoint());
			mp.setPoint(null, null);
			mp.getObservations().clear();
			mp.computePrincipleObservation();
			mp.setFrameTriangulated(null);
			this.map.getUncheckedMapPoints().remove(mp);
		}
	}

	public List<Keyframe> getLastNKeyframesByParallax(int n, double spacing) {
		List<Keyframe> lastKeyframes = new ArrayList<Keyframe>();

		if (this.map.getCurrentKeyframe() != null) {
			lastKeyframes.add(this.map.getCurrentKeyframe());
			Keyframe kf = lastKeyframes.get(0).getPreviousKeyframe();
			for (int i = 0; i < n - 1 && kf != null; i++) {
				if (lastKeyframes.get(i).getPose().getDistanceFrom(kf.getPose()) > spacing
						&& this.numWithHighParallax(lastKeyframes.get(i), kf) > 15) {
					lastKeyframes.add(kf);
				} else {
					i--;
				}
				kf = kf.getPreviousKeyframe();
			}
		}

		return lastKeyframes;
	}

	// return the number of points shared between these keyframes with high parallax
	public int numWithHighParallax(Keyframe kf, Keyframe kfPrev) {

		double GOOD_PARALLAX = 1.0;

		// get overlapping map points
		HashMap<MapPoint, Boolean> mapPointTable = new HashMap<MapPoint, Boolean>();

		for (int i = 0; i < kf.getMapPoints().size(); i++) {
			mapPointTable.put(kf.getMapPoints().get(i), false);
		}

		for (int i = 0; i < kfPrev.getMapPoints().size(); i++) {
			if (mapPointTable.get(kfPrev.getMapPoints().get(i)) != null) {
				mapPointTable.put(kfPrev.getMapPoints().get(i), true);
			}
		}

		List<MapPoint> commonMapPoints = new ArrayList<MapPoint>();
		for (MapPoint mp : mapPointTable.keySet()) {
			if (mapPointTable.get(mp)) {
				commonMapPoints.add(mp);
			}
		}

		// get triangulated map points
		List<Point3D> points = new ArrayList<Point3D>();
		synchronized (this.map) {
			for (int i = 0; i < commonMapPoints.size(); i++) {
				if (commonMapPoints.get(i).getPoint() != null) {
					points.add(commonMapPoints.get(i).getPoint());
				}
			}
		}

		// check parallax of each point
		Pose pose0 = kf.getPose();
		Pose pose1 = kfPrev.getPose();
		int numGoodParallax = 0;
		for (int i = 0; i < points.size(); i++) {
			Point3D pt = points.get(i);
			double ux = pose0.getCx() - pt.getX();
			double uy = pose0.getCy() - pt.getY();
			double uz = pose0.getCz() - pt.getZ();
			double vx = pose1.getCx() - pt.getX();
			double vy = pose1.getCy() - pt.getY();
			double vz = pose1.getCz() - pt.getZ();
			double uMag = Math.sqrt(ux * ux + uy * uy + uz * uz);
			double vMag = Math.sqrt(vx * vx + vy * vy + vz * vz);
			double uDotV = ux * vx + uy * vy + uz * vz;
			double cosParallax = uDotV / (uMag * vMag);
			double parallax = Math.acos(cosParallax) * 180 / Math.PI;
			numGoodParallax += parallax >= GOOD_PARALLAX ? 1 : 0;
		}

		Utils.pl("numGoodParallax: " + numGoodParallax);

		return numGoodParallax;

	}

	public Keyframe getBestKeyframeForPairBA(Pose pose, List<MapPoint> mapPoints,
			List<Correspondence2D2D> correspondences, List<MapPoint> outMapPoints,
			List<Correspondence2D2D> outCorrespondences, int searchLimit, int numGoodPoints) {

		double GOOD_PARALLAX = 0.5;

		Keyframe bestKF = null;
		Keyframe kf = this.map.getCurrentKeyframe();
		boolean keepGoing = true;
		int iterations = 0;
		while (keepGoing) {

			iterations++;

			// get overlapping map points
			List<MapPoint> commonMapPoints = new ArrayList<MapPoint>();
			List<Correspondence2D2D> commonCorrespondences = new ArrayList<Correspondence2D2D>();
			for (int i = 0; i < mapPoints.size(); i++) {
				if (kf.getMapPointIndices().get(mapPoints.get(i)) != null) {
					commonMapPoints.add(mapPoints.get(i));
					commonCorrespondences.add(correspondences.get(i));
				}
			}

			// extract point3Ds
			List<Point3D> points = new ArrayList<Point3D>();
			outMapPoints.clear();
			List<Correspondence2D2D> pointCorrespondences = new ArrayList<Correspondence2D2D>();
			synchronized (this.map) {
				for (int i = 0; i < commonMapPoints.size(); i++) {
					if (commonMapPoints.get(i).getPoint() != null) {
						points.add(commonMapPoints.get(i).getPoint());
						pointCorrespondences.add(new Correspondence2D2D(commonCorrespondences.get(i)));
						outMapPoints.add(commonMapPoints.get(i));
					}
				}
			}

			// check parallax for points
			Pose pose0 = pose;
			Pose pose1 = kf.getPose();
			int numGoodParallax = 0;
			for (int i = 0; i < points.size(); i++) {
				Point3D pt = points.get(i);
				double ux = pose0.getCx() - pt.getX();
				double uy = pose0.getCy() - pt.getY();
				double uz = pose0.getCz() - pt.getZ();
				double vx = pose1.getCx() - pt.getX();
				double vy = pose1.getCy() - pt.getY();
				double vz = pose1.getCz() - pt.getZ();
				double uMag = Math.sqrt(ux * ux + uy * uy + uz * uz);
				double vMag = Math.sqrt(vx * vx + vy * vy + vz * vz);
				double uDotV = ux * vx + uy * vy + uz * vz;
				double cosParallax = uDotV / (uMag * vMag);
				double parallax = Math.acos(cosParallax) * 180 / Math.PI;
				numGoodParallax += parallax >= GOOD_PARALLAX ? 1 : 0;
			}

			if (numGoodParallax >= numGoodPoints) {
				keepGoing = false;
				bestKF = kf;
				outCorrespondences.clear();
				outCorrespondences.addAll(pointCorrespondences);
			} else if (kf.getPreviousKeyframe() != null) {
				kf = kf.getPreviousKeyframe();
			} else {
				keepGoing = false;
			}

			if (iterations > searchLimit) {
				keepGoing = false;
			}

		}

		// update correspondences
		for (int i = 0; i < outCorrespondences.size(); i++) {
			Correspondence2D2D c = outCorrespondences.get(i);
			MapPoint mp = outMapPoints.get(i);
			int index = bestKF.getMapPointIndices().get(mp).get(0);
			KeyPoint kp = bestKF.getKeypointsList().get(index);
			c.setX0(kp.pt.x);
			c.setY0(kp.pt.y);
		}

		return bestKF;

	}

	public List<Keyframe> getLastNKeyframes(int n, double spacing) {
		List<Keyframe> lastKeyframes = new ArrayList<Keyframe>();

		if (this.map.getCurrentKeyframe() != null) {
			lastKeyframes.add(this.map.getCurrentKeyframe());
			Keyframe kf = lastKeyframes.get(0).getPreviousKeyframe();
			for (int i = 0; i < n - 1 && kf != null; i++) {
				if (lastKeyframes.get(i).getPose().getDistanceFrom(kf.getPose()) > spacing) {
					lastKeyframes.add(kf);
				} else {
					i--;
				}
				kf = kf.getPreviousKeyframe();
			}
		}

		return lastKeyframes;
	}

	// for the given keyframes get reprojection errors for points and their
	// observations. remove observations with high reprojection error, retriangulate
	// points if possible (if not, remove them too)
	public void pruneOutliers(List<Keyframe> keyframes) {

		if (this.map.isLoopBeingClosed()) {
			return;
		}

		double initializationReprojectionError = Parameters.<Double>get("initializationReprojectionError");

		// start with statistics
		for (int i = 0; i < keyframes.size(); i++) {
			List<Double> errors = new ArrayList<Double>();
			List<MapPoint> mps = keyframes.get(i).getMapPoints();
			for (int j = 0; j < mps.size(); j++) {
				MapPoint mp = mps.get(j);
				Point3D pt3 = mp.getPoint();
				if (pt3 == null || mp.getAssociatedObject() != null)
					continue;
				List<Observation> obsvs = mp.getObservations();
				boolean prunedObsv = false;
				for (int k = 0; k < obsvs.size(); k++) {
					Observation obsv = obsvs.get(k);

					// get reprojection error
					double reprojErr = Photogrammetry.getReprojectionError(obsv.getKeyframe().getPose(), pt3,
							obsv.getPoint(), Parameters.getK4x4());

					errors.add(reprojErr);

					// if error too high, prune the observation
					if (reprojErr > PRUNING_THRESHOLD) {
						synchronized (this.map) {
							mp.getObservations().remove(obsv);
							this.map.getMapPointsNeedingUpdatedPD().put(mp, true);
							prunedObsv = true;
							totalObsvPruned++;
						}
					}

				}

				// if observations were pruned, check if the 3D point need to be removed or if
				// it needs to be retriangulated
				if (prunedObsv) {
					HashMap<Keyframe, List<Observation>> keyframesInObservations = new HashMap<Keyframe, List<Observation>>();
					for (int k = 0; k < obsvs.size(); k++) {
						if (keyframesInObservations.get(obsvs.get(k).getKeyframe()) == null) {
							keyframesInObservations.put(obsvs.get(k).getKeyframe(), new ArrayList<Observation>());
						}
						keyframesInObservations.get(obsvs.get(k).getKeyframe()).add(obsvs.get(k));
					}
					if (keyframesInObservations.keySet().size() < 2) {
						synchronized (this.map) {
							map.unPartitionMapPoint(mp);
							mp.setPoint(null, null);
							this.map.getAllPoints().remove(pt3);
							totalPointsPruned++;
						}
					} else {
						// retriangulate?

						// check baseline of 2 of the keyframes
						List<Keyframe> keyframesList = new ArrayList<Keyframe>();
						keyframesList.addAll(keyframesInObservations.keySet());

						Keyframe kf0 = keyframesList.get(0);
						Keyframe kf1 = keyframesList.get(keyframesList.size() - 1);

						// if not enough baseline, remove the point
						if (kf0.getPose().getDistanceFrom(kf1.getPose()) < 1) {
							synchronized (this.map) {
								map.unPartitionMapPoint(mp);
								mp.setPoint(null, null);
								this.map.getAllPoints().remove(pt3);
								totalPointsPruned++;
							}
							continue;
						}

						Observation o0 = keyframesInObservations.get(kf0).get(0);
						Observation o1 = keyframesInObservations.get(kf1).get(0);
						Correspondence2D2D c = new Correspondence2D2D();
						c.setX0(o0.getPoint().x);
						c.setY0(o0.getPoint().y);
						c.setX1(o1.getPoint().x);
						c.setY1(o1.getPoint().y);

						Dbl parallax = new Dbl(0);
						Dbl err0 = new Dbl(0);
						Dbl err1 = new Dbl(0);
						Dbl w0 = new Dbl(0);
						Dbl w1 = new Dbl(0);
						Matrix newPoint = Photogrammetry.triangulate(o1.getKeyframe().getPose().getHomogeneousMatrix(),
								o0.getKeyframe().getPose().getHomogeneousMatrix(), c, parallax, err1, err0, w1, w0);

						// if the point does not have enough parallax, is behind the cameras, or has bad
						// reprojection error, remove the point
						if (parallax.getValue() < 1.0 || w0.getValue() < 0 || w1.getValue() < 0
								|| err0.getValue() > initializationReprojectionError
								|| err1.getValue() > initializationReprojectionError) {
							synchronized (this.map) {
								map.unPartitionMapPoint(mp);
								mp.setPoint(null, null);
								this.map.getAllPoints().remove(pt3);
								totalPointsPruned++;
							}
							continue;
						}

						// otherwise, reset the map point
						synchronized (this.map) {
							map.unPartitionMapPoint(mp);
							pt3.setX(newPoint.get(0, 0));
							pt3.setY(newPoint.get(1, 0));
							pt3.setZ(newPoint.get(2, 0));
							map.partitionMapPoint(mp);
							totalToTriangulate++;
						}

					}
				}
			}

		}

		Utils.pl("pruned: " + totalPointsPruned);
		Utils.pl("retriangulated: " + totalToTriangulate);

	}

	public void windowedBundleAdjustment(List<Keyframe> keyframes, int iterations) throws Exception {
		this.windowedBundleAdjustment(keyframes, iterations, true);
	}

	public void windowedBundleAdjustment(List<Keyframe> keyframes, int iterations, boolean checkForLoopClosure)
			throws Exception {

		// if loop closure is in progress, skip BA
		if (checkForLoopClosure && this.map.isLoopBeingClosed()) {
			return;
		}

		// record current loop closure status, in case a loop closure happens in the
		// middle of BA
		long lastLoopClosedFrame = this.map.getLastLoopClosed();

		SceneStructureMetric scene = new SceneStructureMetric(false);
		SceneObservations sceneObservations = new SceneObservations();

		HashMap<Keyframe, Integer> keyframeIndices = new HashMap<Keyframe, Integer>();
		List<Point3D> point3Ds = new ArrayList<Point3D>();
		List<MapPoint> capturedMapPoints = new ArrayList<MapPoint>();
		List<List<Keyframe>> keyframesPerPoint3D = new ArrayList<List<Keyframe>>();
		List<Observation> observations = new ArrayList<Observation>();
		List<Integer> observationToPoint3DIndex = new ArrayList<Integer>();

		// capture map data and load into BoofCV
		synchronized (this.map) {

			// collect common map points between keyframes
			HashMap<MapPoint, Boolean> commonMapPointTable = new HashMap<MapPoint, Boolean>();
			for (int i = 0; i < keyframes.size(); i++) {
				for (int j = 0; j < keyframes.get(i).getMapPoints().size(); j++) {
					MapPoint mp = keyframes.get(i).getMapPoints().get(j);
					if (mp.getAssociatedObject() != null) {
						continue;
					}
					if (commonMapPointTable.get(mp) == null) {
						commonMapPointTable.put(mp, false);
					} else {
						commonMapPointTable.put(mp, true);
					}
				}
			}
			List<MapPoint> mapPoints = new ArrayList<MapPoint>();
			for (MapPoint mp : commonMapPointTable.keySet()) {
				if (commonMapPointTable.get(mp)) {
					mapPoints.add(mp);
				}
			}

			// ---- capture data ---- //
			for (int i = 0; i < keyframes.size(); i++) {
				keyframeIndices.put(keyframes.get(i), i);
			}
			for (int i = 0; i < mapPoints.size(); i++) {
				MapPoint mp = mapPoints.get(i);
				if (mp.getPoint() == null) {
					continue;
				}
				HashMap<Keyframe, Boolean> associatedKeyframes = new HashMap<Keyframe, Boolean>();
				point3Ds.add(mp.getPoint());
				capturedMapPoints.add(mp);
				for (int j = 0; j < mp.getObservations().size(); j++) {

					// ensure that the observation is related to a keyframe in the window
					if (keyframeIndices.get(mp.getObservations().get(j).getKeyframe()) == null) {
						continue;
					}

					observations.add(mp.getObservations().get(j));
					observationToPoint3DIndex.add(point3Ds.size() - 1);
					associatedKeyframes.put(mp.getObservations().get(j).getKeyframe(), true);
				}
				List<Keyframe> associatedKeyframesList = new ArrayList<Keyframe>();
				associatedKeyframesList.addAll(associatedKeyframes.keySet());
				keyframesPerPoint3D.add(associatedKeyframesList);
			}

			// ---- load data into BoofCV ---- //
			scene.initialize(keyframes.size(), keyframes.size(), point3Ds.size());
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
				scene.setView(i, keyframes.get(i).getPose().isFixed(), worldToCameraGL);
				scene.connectViewToCamera(i, i);
			}

			// load points
			Utils.pl("point3Ds.size(): " + point3Ds.size());
			for (int i = 0; i < point3Ds.size(); i++) {
				float x = (float) point3Ds.get(i).getX();
				float y = (float) point3Ds.get(i).getY();
				float z = (float) point3Ds.get(i).getZ();
				scene.setPoint(i, x, y, z);
				for (int j = 0; j < keyframesPerPoint3D.get(i).size(); j++) {
					scene.getPoints().get(i).views.add(keyframeIndices.get(keyframesPerPoint3D.get(i).get(j)));
				}

			}

			// load observations
			Utils.pl("observations.size(): " + observations.size());
			for (int i = 0; i < observations.size(); i++) {
				Observation o = observations.get(i);
				int viewID = keyframeIndices.get(o.getKeyframe());
				int pointID = observationToPoint3DIndex.get(i);
				sceneObservations.getView(viewID).add(pointID, (float) o.getPoint().x, (float) o.getPoint().y);
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
				bundleAdjustScene(scene, sceneObservations, iterations);
				BAIncomplete = false;
			} catch (Exception e) {
				if (i >= maxAttempts) {
					throw e;
				}
				Utils.pl(
						"windowed BA failed, attempting to prune... !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
				pruner.pruneObservationsByErrorRank(ITERATIVE_PRUNING_INLIER_THRESHOLD);
			}
		}

		// lock the map and update the points
		synchronized (this.map) {

			// double check for loop closure cancellation
			if (checkForLoopClosure
					&& (this.map.isLoopBeingClosed() || lastLoopClosedFrame != this.map.getLastLoopClosed())) {
				return;
			}

			// load points from scene back into input
			for (int i = 0; i < scene.getPoints().size(); i++) {
				point3Ds.get(i).setX(scene.getPoints().get(i).getX());
				point3Ds.get(i).setY(scene.getPoints().get(i).getY());
				point3Ds.get(i).setZ(scene.getPoints().get(i).getZ());
				MapPoint mp = capturedMapPoints.get(i);
				if (mp.getAssociatedObject() != null) {
					this.map.unPartitionMapPoint(mp);
					this.map.partitionMapPoint(mp);
				}
			}

			// load poses from scene back into input
			for (int viewID = 0; viewID < keyframes.size(); viewID++) {
				Se3_F64 worldToView = scene.getViews().get(viewID).worldToView;
				Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(worldToView.getR(), null);
				q.normalize();
				Vector3D_F64 t = worldToView.getTranslation();
				keyframes.get(viewID).getPose().setQw(q.w);
				keyframes.get(viewID).getPose().setQx(q.x);
				keyframes.get(viewID).getPose().setQy(q.y);
				keyframes.get(viewID).getPose().setQz(q.z);
				keyframes.get(viewID).getPose().setT(t.x, t.y, t.z);
			}
		}

	}

	public boolean fixedWindowBundleAdjustment(List<Keyframe> keyframes, boolean fixFirstKeyframe,
			boolean fixLastKeyframe, int iterations, boolean checkForLoopClosure) throws Exception {

		// if loop closure is in progress, skip BA
		if (checkForLoopClosure && this.map.isLoopBeingClosed()) {
			return false;
		}

		// record current loop closure status, in case a loop closure happens in the
		// middle of BA
		long lastLoopClosedFrame = this.map.getLastLoopClosed();

		SceneStructureMetric scene = new SceneStructureMetric(false);
		SceneObservations sceneObservations = new SceneObservations();

		HashMap<Keyframe, Integer> keyframeIndices = new HashMap<Keyframe, Integer>();
		List<Point3D> point3Ds = new ArrayList<Point3D>();
		List<MapPoint> capturedMapPoints = new ArrayList<MapPoint>();
		List<List<Keyframe>> keyframesPerPoint3D = new ArrayList<List<Keyframe>>();
		List<Observation> observations = new ArrayList<Observation>();
		List<Integer> observationToPoint3DIndex = new ArrayList<Integer>();

		// capture map data and load into BoofCV
		synchronized (this.map) {

			// collect common map points between keyframes
			HashMap<MapPoint, Boolean> commonMapPointTable = new HashMap<MapPoint, Boolean>();
			for (int i = 0; i < keyframes.size(); i++) {
				for (int j = 0; j < keyframes.get(i).getMapPoints().size(); j++) {
					MapPoint mp = keyframes.get(i).getMapPoints().get(j);
					if (mp.getAssociatedObject() != null) {
						continue;
					}
					if (commonMapPointTable.get(mp) == null) {
						commonMapPointTable.put(mp, false);
					} else {
						commonMapPointTable.put(mp, true);
					}
				}
			}
			List<MapPoint> mapPoints = new ArrayList<MapPoint>();
			for (MapPoint mp : commonMapPointTable.keySet()) {
				if (commonMapPointTable.get(mp)) {
					mapPoints.add(mp);
				}
			}

			// ---- capture data ---- //
			for (int i = 0; i < keyframes.size(); i++) {
				keyframeIndices.put(keyframes.get(i), i);
			}
			for (int i = 0; i < mapPoints.size(); i++) {
				MapPoint mp = mapPoints.get(i);
				if (mp.getPoint() == null) {
					continue;
				}
				HashMap<Keyframe, Boolean> associatedKeyframes = new HashMap<Keyframe, Boolean>();
				point3Ds.add(mp.getPoint());
				capturedMapPoints.add(mp);
				for (int j = 0; j < mp.getObservations().size(); j++) {

					// ensure that the observation is related to a keyframe in the window
					if (keyframeIndices.get(mp.getObservations().get(j).getKeyframe()) == null) {
						continue;
					}

					observations.add(mp.getObservations().get(j));
					observationToPoint3DIndex.add(point3Ds.size() - 1);
					associatedKeyframes.put(mp.getObservations().get(j).getKeyframe(), true);
				}
				List<Keyframe> associatedKeyframesList = new ArrayList<Keyframe>();
				associatedKeyframesList.addAll(associatedKeyframes.keySet());
				keyframesPerPoint3D.add(associatedKeyframesList);
			}

			// ---- load data into BoofCV ---- //
			scene.initialize(keyframes.size(), keyframes.size(), point3Ds.size());
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
				boolean fixed = keyframes.get(i).getPose().isFixed();
				fixed = i == 0 && fixFirstKeyframe || i == keyframes.size() - 1 && fixLastKeyframe ? true : fixed;
				scene.setView(i, fixed, worldToCameraGL);
				scene.connectViewToCamera(i, i);
			}

			// load points
			Utils.pl("point3Ds.size(): " + point3Ds.size());
			for (int i = 0; i < point3Ds.size(); i++) {
				float x = (float) point3Ds.get(i).getX();
				float y = (float) point3Ds.get(i).getY();
				float z = (float) point3Ds.get(i).getZ();
				scene.setPoint(i, x, y, z);
				for (int j = 0; j < keyframesPerPoint3D.get(i).size(); j++) {
					scene.getPoints().get(i).views.add(keyframeIndices.get(keyframesPerPoint3D.get(i).get(j)));
				}

			}

			// load observations
			Utils.pl("observations.size(): " + observations.size());
			for (int i = 0; i < observations.size(); i++) {
				Observation o = observations.get(i);
				int viewID = keyframeIndices.get(o.getKeyframe());
				int pointID = observationToPoint3DIndex.get(i);
				sceneObservations.getView(viewID).add(pointID, (float) o.getPoint().x, (float) o.getPoint().y);
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
				bundleAdjustScene(scene, sceneObservations, iterations);
				BAIncomplete = false;
			} catch (Exception e) {
				if (i >= maxAttempts) {
					throw e;
				}
				Utils.pl(
						"windowed BA failed, attempting to prune... !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
				pruner.pruneObservationsByErrorRank(ITERATIVE_PRUNING_INLIER_THRESHOLD);
			}
		}

		// lock the map and update the points
		synchronized (this.map) {

			// double check for loop closure cancellation
			if (checkForLoopClosure
					&& (this.map.isLoopBeingClosed() || lastLoopClosedFrame != this.map.getLastLoopClosed())) {
				return false;
			}

			// load points from scene back into input
			for (int i = 0; i < scene.getPoints().size(); i++) {
				point3Ds.get(i).setX(scene.getPoints().get(i).getX());
				point3Ds.get(i).setY(scene.getPoints().get(i).getY());
				point3Ds.get(i).setZ(scene.getPoints().get(i).getZ());
				MapPoint mp = capturedMapPoints.get(i);
				if (mp.getAssociatedObject() != null) {
					this.map.unPartitionMapPoint(mp);
					this.map.partitionMapPoint(mp);
				}
			}

			// load poses from scene back into input
			for (int viewID = 0; viewID < keyframes.size(); viewID++) {
				Se3_F64 worldToView = scene.getViews().get(viewID).worldToView;
				Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(worldToView.getR(), null);
				q.normalize();
				Vector3D_F64 t = worldToView.getTranslation();
				keyframes.get(viewID).getPose().setQw(q.w);
				keyframes.get(viewID).getPose().setQx(q.x);
				keyframes.get(viewID).getPose().setQy(q.y);
				keyframes.get(viewID).getPose().setQz(q.z);
				keyframes.get(viewID).getPose().setT(t.x, t.y, t.z);
			}
		}

		return true;

	}

	public void fullBundleAdjustment(int iterations) throws Exception {

		SceneStructureMetric scene = new SceneStructureMetric(false);
		SceneObservations sceneObservations = new SceneObservations();

		HashMap<Keyframe, Integer> keyframeIndices = new HashMap<Keyframe, Integer>();
		List<Keyframe> keyframes = new ArrayList<Keyframe>();
		List<Point3D> point3Ds = new ArrayList<Point3D>();
		List<MapPoint> capturedMapPoints = new ArrayList<MapPoint>();
		List<List<Keyframe>> keyframesPerPoint3D = new ArrayList<List<Keyframe>>();
		List<Observation> observations = new ArrayList<Observation>();
		List<Integer> observationToPoint3DIndex = new ArrayList<Integer>();

		// capture map data and load into BoofCV
		synchronized (this.map) {

			// ---- capture data ---- //
			keyframes = this.map.getKeyframes();
			for (int i = 0; i < keyframes.size(); i++) {
				keyframeIndices.put(keyframes.get(i), i);
			}
			for (int i = 0; i < this.map.getAllMapPoints().size(); i++) {
				MapPoint mp = this.map.getAllMapPoints().get(i);
				if (mp.getPoint() == null || mp.getAssociatedObject() != null) {
					continue;
				}
				HashMap<Keyframe, Boolean> associatedKeyframes = new HashMap<Keyframe, Boolean>();
				point3Ds.add(mp.getPoint());
				capturedMapPoints.add(mp);
				for (int j = 0; j < mp.getObservations().size(); j++) {
					observations.add(mp.getObservations().get(j));
					observationToPoint3DIndex.add(point3Ds.size() - 1);
					associatedKeyframes.put(mp.getObservations().get(j).getKeyframe(), true);
				}
				List<Keyframe> associatedKeyframesList = new ArrayList<Keyframe>();
				associatedKeyframesList.addAll(associatedKeyframes.keySet());
				keyframesPerPoint3D.add(associatedKeyframesList);
			}

			// ---- load data into BoofCV ---- //
			scene.initialize(keyframes.size(), keyframes.size(), point3Ds.size());
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
				scene.setView(i, keyframes.get(i).getPose().isFixed(), worldToCameraGL);
				scene.connectViewToCamera(i, i);
			}

			// load points
			Utils.pl("point3Ds.size(): " + point3Ds.size());
			for (int i = 0; i < point3Ds.size(); i++) {
				float x = (float) point3Ds.get(i).getX();
				float y = (float) point3Ds.get(i).getY();
				float z = (float) point3Ds.get(i).getZ();
				scene.setPoint(i, x, y, z);
				for (int j = 0; j < keyframesPerPoint3D.get(i).size(); j++) {
					scene.getPoints().get(i).views.add(keyframeIndices.get(keyframesPerPoint3D.get(i).get(j)));
				}

			}

			// load observations
			for (int i = 0; i < observations.size(); i++) {
				Observation o = observations.get(i);
				int viewID = keyframeIndices.get(o.getKeyframe());
				int pointID = observationToPoint3DIndex.get(i);
				sceneObservations.getView(viewID).add(pointID, (float) o.getPoint().x, (float) o.getPoint().y);
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
				bundleAdjustScene(scene, sceneObservations, iterations);
				BAIncomplete = false;
			} catch (Exception e) {
				if (i >= maxAttempts) {
					throw e;
				}
				Utils.pl(
						"full BA failed, attempting to prune... !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
				pruner.pruneObservationsByErrorRank(ITERATIVE_PRUNING_INLIER_THRESHOLD);
			}
		}

		// lock the map and update the points
		synchronized (this.map) {

			// load points from scene back into input
			for (int i = 0; i < scene.getPoints().size(); i++) {
				point3Ds.get(i).setX(scene.getPoints().get(i).getX());
				point3Ds.get(i).setY(scene.getPoints().get(i).getY());
				point3Ds.get(i).setZ(scene.getPoints().get(i).getZ());
				MapPoint mp = capturedMapPoints.get(i);
				this.map.unPartitionMapPoint(mp);
				this.map.partitionMapPoint(mp);
			}

			// load poses from scene back into input
			for (int viewID = 0; viewID < keyframes.size(); viewID++) {
				Se3_F64 worldToView = scene.getViews().get(viewID).worldToView;
				Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(worldToView.getR(), null);
				q.normalize();
				Vector3D_F64 t = worldToView.getTranslation();
				keyframes.get(viewID).getPose().setQw(q.w);
				keyframes.get(viewID).getPose().setQx(q.x);
				keyframes.get(viewID).getPose().setQy(q.y);
				keyframes.get(viewID).getPose().setQz(q.z);
				keyframes.get(viewID).getPose().setT(t.x, t.y, t.z);
			}
		}

	}

	public void pairBundleAdjustment(Pose currentPose, Pose keyframePose, List<MapPoint> matchedMapPoints,
			List<Correspondence2D2D> correspondences, int iterations) throws Exception {
		pairBundleAdjustment(currentPose, keyframePose, matchedMapPoints, correspondences, iterations, false, false);
	}

	// perform bundle adjustment for the current pose and the current keyframe pose
	public void pairBundleAdjustment(Pose currentPose, Pose keyframePose, List<MapPoint> matchedMapPoints,
			List<Correspondence2D2D> correspondences, int iterations, boolean lockPoints, boolean ignoreLoopClosure)
			throws Exception {

		// if loop closure is in progress, skip BA
		if (this.map.isLoopBeingClosed() && !ignoreLoopClosure) {
			return;
		}

		// record current loop closure status, in case a loop closure happens in the
		// middle of BA
		long lastLoopClosedFrame = this.map.getLastLoopClosed();

		SceneStructureMetric scene = new SceneStructureMetric(false);
		SceneObservations observations = new SceneObservations();
		observations.initialize(2);

		// load camera poses into scene
		BundlePinhole camera = new BundlePinhole();
		camera.fx = Parameters.<Float>get("fx").doubleValue();
		camera.fy = Parameters.<Float>get("fy").doubleValue();
		camera.cx = Parameters.<Float>get("cx").doubleValue();
		camera.cy = Parameters.<Float>get("cy").doubleValue();
		camera.skew = Parameters.<Float>get("s").doubleValue();

		// init camera list
		List<Pose> cameras = new ArrayList<Pose>();

		// init pruned lists for points
		List<Correspondence2D2D> prunedCorrespondences = new ArrayList<Correspondence2D2D>();
		List<Point3D> prunedPoints = new ArrayList<Point3D>();
		List<MapPoint> capturedMapPoints = new ArrayList<MapPoint>();

		// lock the map and load the scene
		synchronized (this.map) {

			// prune untracked points and load into lists
			for (int i = 0; i < matchedMapPoints.size(); i++) {
				if (matchedMapPoints.get(i).getPoint() == null) {
					continue;
				}
				prunedCorrespondences.add(correspondences.get(i));
				prunedPoints.add(matchedMapPoints.get(i).getPoint());
				capturedMapPoints.add(matchedMapPoints.get(i));
			}

			// initialize the scene
			scene.initialize(2, 2, prunedPoints.size());

			// load cameras/views
			cameras.add(keyframePose);
			cameras.add(currentPose);

			for (int i = 0; i < cameras.size(); i++) {
				Se3_F64 worldToCameraGL = new Se3_F64();
				ConvertRotation3D_F64.quaternionToMatrix(cameras.get(i).getQw(), cameras.get(i).getQx(),
						cameras.get(i).getQy(), cameras.get(i).getQz(), worldToCameraGL.R);
				worldToCameraGL.T.x = cameras.get(i).getTx();
				worldToCameraGL.T.y = cameras.get(i).getTy();
				worldToCameraGL.T.z = cameras.get(i).getTz();
				scene.setCamera(i, true, camera);
				scene.setView(i, i == 0 ? true : false, worldToCameraGL);
				scene.connectViewToCamera(i, i);
			}

			// load observations to scene
			for (int i = 0; i < prunedCorrespondences.size(); i++) {
				Correspondence2D2D c = prunedCorrespondences.get(i);
				observations.getView(0).add(i, (float) c.getX0(), (float) c.getY0());
				observations.getView(1).add(i, (float) c.getX1(), (float) c.getY1());
			}

			// load 3D points into scene
			for (int i = 0; i < prunedPoints.size(); i++) {
				float x = (float) prunedPoints.get(i).getX();
				float y = (float) prunedPoints.get(i).getY();
				float z = (float) prunedPoints.get(i).getZ();
				scene.setPoint(i, x, y, z);
				scene.getPoints().get(i).views.add(0);
				scene.getPoints().get(i).views.add(1);
			}

		}

		// omit top x% of observations with highest reprojection error
		PruneStructureFromSceneMetric pruner = new PruneStructureFromSceneMetric(scene, observations);
		pruner.pruneObservationsByErrorRank(PRUNING_INLIER_THRESHOLD);

		// perform bundle adjustment
		bundleAdjustScene(scene, observations, iterations);

		// lock the map and update the points
		synchronized (this.map) {

			// double check for loop closure cancellation
			if (!ignoreLoopClosure
					&& (this.map.isLoopBeingClosed() || lastLoopClosedFrame != this.map.getLastLoopClosed())) {
				return;
			}

			// load points from scene back into input
			if (!lockPoints) {
				for (int i = 0; i < scene.getPoints().size(); i++) {
					prunedPoints.get(i).setX(scene.getPoints().get(i).getX());
					prunedPoints.get(i).setY(scene.getPoints().get(i).getY());
					prunedPoints.get(i).setZ(scene.getPoints().get(i).getZ());
					MapPoint mp = capturedMapPoints.get(i);
					if (mp.getAssociatedObject() != null) {
						this.map.unPartitionMapPoint(mp);
						this.map.partitionMapPoint(mp);
					}
				}
			}

			// load poses from scene back into input
			for (int viewID = 0; viewID < cameras.size(); viewID++) {
				Se3_F64 worldToView = scene.getViews().get(viewID).worldToView;
				Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(worldToView.getR(), null);
				q.normalize();
				Vector3D_F64 t = worldToView.getTranslation();
				cameras.get(viewID).setQw(q.w);
				cameras.get(viewID).setQx(q.x);
				cameras.get(viewID).setQy(q.y);
				cameras.get(viewID).setQz(q.z);
				cameras.get(viewID).setT(t.x, t.y, t.z);
			}
		}

	}

	public static void bundleAdjustScene(SceneStructureMetric scene, SceneObservations observations, int maxIterations)
			throws Exception {
		bundleAdjustScene(scene, observations, maxIterations, false);
	}

	public static void bundleAdjustScene(SceneStructureMetric scene, SceneObservations observations, int maxIterations,
			boolean showErrors) throws Exception {

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
		if (showErrors) {
			Utils.pl("error before: " + errorBefore);
		}
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
		if (showErrors) {

			System.out.println();
			System.out.printf("Error reduced by %.1f%%\n",
					(100.0 * (errorBefore / bundleAdjustment.getFitScore() - 1.0)));
			System.out.println("new error: " + bundleAdjustment.getFitScore());
			System.out.println((System.currentTimeMillis() - startTime) / 1000.0);
		}

		// Return parameters to their original scaling. Can probably skip this
		// step.
		bundleScale.undoScale(scene, observations);

	}

	public Map getMap() {
		return map;
	}

	public void setMap(Map map) {
		this.map = map;
	}

}
