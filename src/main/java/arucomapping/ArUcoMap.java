package arucomapping;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import org.apache.commons.io.FileUtils;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Point3;

import Jama.Matrix;
import lumoslam.Keyframe;
import lumoslam.Photogrammetry;
import runtimevars.Parameters;
import toolbox.Utils;
import types.Dbl;
import types.Point3D;
import types.Pose;
import types.Transformation;

public class ArUcoMap {

	protected ArUcoMapOptimizer optimizer = new ArUcoMapOptimizer(this);
	protected boolean localized = false;
	protected Transformation transformation = new Transformation();
	protected double scale = 1;
	protected List<ArUcoMapPoint> mapPoints = new ArrayList<ArUcoMapPoint>();
	protected HashMap<String, ArUcoMapPoint> mapPointByArUco = new HashMap<String, ArUcoMapPoint>();

	public ArUcoMap() {

	}

	public ArUcoMap(String mapFile, List<List<Integer>> movingObjectIds) {
		this.loadMap(mapFile, movingObjectIds);
	}

	public void loadMap(String mapFile, List<List<Integer>> movingObjectIds) {

		String csv = "";
		try {
			csv = FileUtils.readFileToString(new File(mapFile), StandardCharsets.US_ASCII);
		} catch (IOException e) {
			e.printStackTrace();
			return;
		}

		// split into lines
		List<String> lines = new ArrayList(Arrays.asList(csv.split("\n")));

		// discard comments
		for (int i = lines.size() - 1; i >= 0; i--) {
			if (lines.get(i).charAt(0) == '#') {
				lines.remove(i);
			}
		}

		// convert lines to map points
		List<ArUcoMapPoint> allMapPoints = new ArrayList<ArUcoMapPoint>();
		for (String line : lines) {
			List<String> tokens = Arrays.asList(line.split(","));
			String id = tokens.get(0);
			Point3D p = new Point3D();
			p.setX(Double.parseDouble(tokens.get(1)));
			p.setY(Double.parseDouble(tokens.get(2)));
			p.setZ(Double.parseDouble(tokens.get(3)));
			ArUcoMapPoint mp = new ArUcoMapPoint();
			mp.setId(id);
			mp.setPoint(p);
			allMapPoints.add(mp);
		}

		// separate map points based on moving/static
		HashMap<Integer, Integer> movingIds = new HashMap<Integer, Integer>();
		for (int i = 0; i < movingObjectIds.size(); i++) {
			List<Integer> mo = movingObjectIds.get(i);
			for (Integer id : mo) {
				movingIds.put(id, i);
			}
		}

		List<ArUcoMapPoint> staticMPs = new ArrayList<ArUcoMapPoint>();
		List<ArUcoMapPoint> movingMPs = new ArrayList<ArUcoMapPoint>();
		for (ArUcoMapPoint mp : allMapPoints) {
			String[] idSplit = mp.getId().split("-");
			if (movingIds.get(Integer.parseInt(idSplit[0])) != null) {
				movingMPs.add(mp);
			} else {
				staticMPs.add(mp);
			}
		}

		// consolidate moving points into respective models
		List<List<ArUcoMapPoint>> movingModels = new ArrayList<List<ArUcoMapPoint>>();
		for (int i = 0; i < movingObjectIds.size(); i++) {
			movingModels.add(new ArrayList<ArUcoMapPoint>());
		}

		for (ArUcoMapPoint mp : movingMPs) {
			String[] idSplit = mp.getId().split("-");
			movingModels.get(movingIds.get(Integer.parseInt(idSplit[0]))).add(mp);
		}

		// add static map points to map
		for (ArUcoMapPoint mp : staticMPs) {
			this.registerMapPoint(mp);
		}

		// (ignoring moving markers for now, likely will never need to bring them in)

//		// add moving models to map
//		for (int i = 0; i < movingModels.size(); i++) {
//			MovingModel mo = new MovingModel();
//			mo.setId(i + "");
//			mo.setMapPoints(movingModels.get(i));
//			mo.setTransformation(new Transformation());
//			mo.loadMapPointTable();
//			this.movingObjects.add(mo);
//		}

	}

	public void registerObservations(Keyframe keyframe, List<MarkerMPCorrespondence> markerMPCs) {

		for (MarkerMPCorrespondence mmpc : markerMPCs) {
			mmpc.getMapPoint().getObservations().put(keyframe, mmpc.getKeypoint());
		}

	}

	public void attemptLocalization(Pose pose, List<MarkerMPCorrespondence> mpcs) {

		if (mpcs.size() < 6) {
			return;
		}

		Transformation poseReverse = (new Transformation(pose)).getReverseTransformation();

		Pose moPnP = PnP(mpcs, new ArrayList<Integer>(), true, true);
		Transformation objectPose = new Transformation(
				Utils.matrixToPose(poseReverse.getHomogeneousMatrix().times(moPnP.getHomogeneousMatrix())));
		double reprojectionError = mapReprojectionError(mpcs, pose, objectPose);
		if (reprojectionError < 4) {
			this.setTransformation(objectPose);
			this.localized = true;
		}

	}

	public static Pose PnP(List<MarkerMPCorrespondence> mpcs, List<Integer> inlierIndices, boolean optimize,
			boolean useExtrinsicGuess) {

		List<Point3> point3s = new ArrayList<Point3>();
		List<Point> points = new ArrayList<Point>();
		for (MarkerMPCorrespondence mpc : mpcs) {
			Point3D point3D = mpc.getMapPoint().getPoint();
			Point3 point3 = new Point3(point3D.getX(), point3D.getY(), point3D.getZ());
			Point point = new Point(mpc.getKeypoint().locationX, mpc.getKeypoint().locationY);
			point3s.add(point3);
			points.add(point);
		}

		Matrix poseMatrix = Photogrammetry.OpenCVPnP(point3s, points, new Mat(), new Mat(), inlierIndices, optimize,
				useExtrinsicGuess);

		return Utils.matrixToPose(poseMatrix);

	}

	public static double mapReprojectionError(List<MarkerMPCorrespondence> mpcs, Pose pose, Transformation mapPose) {
		// pre-define matrices
		Matrix points = new Matrix(4, mpcs.size(), 1);
		Matrix poseMatrix = pose.getHomogeneousMatrix();
		Matrix objectMatrix = mapPose.getHomogeneousMatrix();
		Matrix K = Parameters.getK4x4();

		// load point matrix
		for (int i = 0; i < mpcs.size(); i++) {
			Point3D p = mpcs.get(i).getMapPoint().getPoint();
			points.set(0, i, p.getX());
			points.set(1, i, p.getY());
			points.set(2, i, p.getZ());
		}

		// get projections
		Matrix projections = K.times(poseMatrix.times(objectMatrix.times(points)));

		// normalize
		for (int i = 0; i < projections.getColumnDimension(); i++) {
			double w = projections.get(2, i);
			double u = projections.get(0, i);
			double v = projections.get(1, i);
			projections.set(0, i, u / w);
			projections.set(1, i, v / w);
		}

		List<Double> reprojErrors = new ArrayList<Double>();
		for (int i = 0; i < mpcs.size(); i++) {
			double obsvX = mpcs.get(i).getKeypoint().locationX;
			double obsvY = mpcs.get(i).getKeypoint().locationY;
			double projX = projections.get(0, i);
			double projY = projections.get(1, i);
			double reprojectionError = Math.sqrt(Math.pow(obsvX - projX, 2) + Math.pow(obsvY - projY, 2));
			reprojErrors.add(reprojectionError);
		}

		Dbl avg = new Dbl(0);
		Dbl stdDev = new Dbl(0);
		Dbl median = new Dbl(0);
		Utils.basicStats(reprojErrors, avg, stdDev, median);

		return avg.getValue();

	}

	public List<MarkerMPCorrespondence> getMapPointCorrespondences(List<ArUcoKeypoint> keypoints) {
		List<MarkerMPCorrespondence> mpc = new ArrayList<MarkerMPCorrespondence>();
		for (ArUcoKeypoint keypoint : keypoints) {
			ArUcoMapPoint mp = this.mapPointByArUco.get(keypoint.id);
			if (mp == null) {
				continue;
			}
			MarkerMPCorrespondence corr = new MarkerMPCorrespondence();
			corr.setKeypoint(keypoint);
			corr.setMapPoint(mp);
			mpc.add(corr);
		}
		return mpc;
	}

	public void registerMapPoint(ArUcoMapPoint mp) {
		this.mapPoints.add(mp);
		this.mapPointByArUco.put(mp.getId(), mp);
	}

	public Transformation getTransformation() {
		return transformation;
	}

	public void setTransformation(Transformation transformation) {
		this.transformation = transformation;
	}

	public List<ArUcoMapPoint> getMapPoints() {
		return mapPoints;
	}

	public void setMapPoints(List<ArUcoMapPoint> mapPoints) {
		this.mapPoints = mapPoints;
	}

	public HashMap<String, ArUcoMapPoint> getMapPointByArUco() {
		return mapPointByArUco;
	}

	public void setMapPointByArUco(HashMap<String, ArUcoMapPoint> mapPointByArUco) {
		this.mapPointByArUco = mapPointByArUco;
	}

	public boolean isLocalized() {
		return localized;
	}

	public void setLocalized(boolean localized) {
		this.localized = localized;
	}

	public double getScale() {
		return scale;
	}

	public void setScale(double scale) {
		this.scale = scale;
	}

	public ArUcoMapOptimizer getOptimizer() {
		return optimizer;
	}

	public void setOptimizer(ArUcoMapOptimizer optimizer) {
		this.optimizer = optimizer;
	}

}
