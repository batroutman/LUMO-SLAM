package lumoslam;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.core.Point3;

import Jama.Matrix;
import runtimevars.Parameters;
import toolbox.Timer;
import toolbox.Utils;
import types.Correspondence2D2D;
import types.Dbl;
import types.Point3D;
import types.Pose;
import types.TrackingException;
import types.Transformation;

public class Tracker {

	protected Map map;

	public Tracker() {

	}

	public Tracker(Map map) {
		this.map = map;
	}

	public class ProjectionInfo {
		public MapPoint mapPoint;
		public Point3D point3D;
		public Point projection;
		public double wValue;
	}

	public class DescriptorInfo {
		public Mat descriptor;
		public KeyPoint keypoint;
		public ProjectionInfo projectionInfo;
	}

	public class PointCorrespondence {
		public boolean outlier;
		public int index;
		public MapPoint mapPoint;
		public Point3D point3D;
		public Matrix point;
		public Correspondence2D2D correspondence;
		public double projX;
		public double projY;
		public double reprojErr;
		public List<PointCorrespondence> neighbors = new ArrayList<PointCorrespondence>();
	}

	public class MatchedPoint {
		public Correspondence2D2D correspondence;
		public MapPoint mapPoint;
		public Point3D point3D;
		public Point3 point3;
		public Point point;
	}

	public void generatePointCorrespondences(List<Correspondence2D2D> correspondences,
			List<MapPoint> correspondenceMapPoints, boolean outliers,
			List<PointCorrespondence> outPointCorrespondences) {
		outPointCorrespondences.clear();
		for (int i = 0; i < correspondences.size(); i++) {
			PointCorrespondence pCorr = new PointCorrespondence();
			pCorr.outlier = outliers;
			pCorr.index = i;
			pCorr.mapPoint = correspondenceMapPoints.get(i);
			pCorr.point3D = pCorr.mapPoint.getPoint();
			if (pCorr.point3D == null) {
				continue;
			}
			Matrix point = new Matrix(4, 1, 1);
			point.set(0, 0, pCorr.point3D.getX());
			point.set(1, 0, pCorr.point3D.getY());
			point.set(2, 0, pCorr.point3D.getZ());
			pCorr.point = point;
			pCorr.correspondence = correspondences.get(i);
			outPointCorrespondences.add(pCorr);
		}
	}

	public void registerMovingObjects3(Pose pose, List<Correspondence2D2D> prunedCorrespondences,
			List<MapPoint> prunedCorrespondenceMapPoints, List<Correspondence2D2D> outlierCorrespondences,
			List<MapPoint> outlierCorrespondenceMapPoints) {

		Timer t = new Timer();

		int MIN_POINTS = 6;

		List<MovingModel> movingObjects = new ArrayList<MovingModel>();
		List<MatchedPoint> matchedPoints = this.buildMatchedPoints(outlierCorrespondences,
				outlierCorrespondenceMapPoints);
		boolean keepGoing = true;
		int attempts = 0;
		while (keepGoing) {
			if (matchedPoints.size() < MIN_POINTS) {
				keepGoing = false;
				continue;
			}

			// prep for PnP
			List<Point3> point3s = new ArrayList<Point3>();
			List<Point> points = new ArrayList<Point>();
			List<Integer> inlierIndices = new ArrayList<Integer>();
			this.spreadMatchedPointList(matchedPoints, point3s, points);

			// -- RANSAC PnP
			Matrix poseMat = new Matrix(4, 4);
			try {
				poseMat = Photogrammetry.OpenCVPnP(point3s, points, new Mat(), new Mat(), inlierIndices, false);
			} catch (Exception e) {
				keepGoing = false;
				continue;
			}

			// check if enough points in estimate
			if (inlierIndices.size() < MIN_POINTS) {
				keepGoing = false;
				continue;
			}

			// there are enough inliers for a model, construct model
			MovingModel mo = new MovingModel();
			Transformation poseReverse = (new Transformation(pose)).getReverseTransformation();
			mo.setTransformation(
					new Transformation(Utils.matrixToPose(poseReverse.getHomogeneousMatrix().times(poseMat))));

			// populate model with points
			HashMap<Integer, Boolean> isInlier = new HashMap<Integer, Boolean>();
			for (Integer idx : inlierIndices) {
				isInlier.put(idx, true);
			}

			// add inliers to MO and remove them from lists
			List<MatchedPoint> inliers = new ArrayList<MatchedPoint>();
			for (int i = matchedPoints.size() - 1; i >= 0; i--) {
				if (isInlier.get(i) == null) {
					continue;
				}
				mo.getMapPoints().add(matchedPoints.get(i).mapPoint);
				inliers.add(matchedPoints.get(i));
				matchedPoints.remove(i);
			}

			if (pointsFulfillConstraints(inliers)) {
				movingObjects.add(mo);
			}

			attempts++;
			if (attempts > Parameters.<Integer>get("maxRegistrationAttempts")) {
				keepGoing = false;
			}
		}

		this.map.registerMovingObjects(movingObjects);

		for (MovingModel mo : movingObjects) {
			this.map.soakUpPoints(mo);
			mo.calculateCentroid();
		}

//		long time = t.stop();
//		if (movingObjects.size() > 0) {
//			Utils.pl("\n\n\n\nMOVING OBJECT REGISTERED: " + time + "ms\n\n\n\n");
//		} else {
//			Utils.pl("\n\n\n\nMOVING OBJECT NOT REGISTERED: " + time + "ms\n\n\n\n");
//		}

	}

	// return true if the collection of points fulfills constraints that indicate it
	// is a valid moving object
	public boolean pointsFulfillConstraints(List<MatchedPoint> moPoints) {

		if (moPoints.size() <= 0) {
			return false;
		}

		// check total spread of points, ensure it is under some threshold
		double MIN_SPREAD = Parameters.<Integer>get("width") * 0.1;
		double MAX_SPREAD = Parameters.<Integer>get("width") * 0.3;
		double minX = moPoints.get(0).point.x;
		double maxX = moPoints.get(0).point.x;
		double minY = moPoints.get(0).point.y;
		double maxY = moPoints.get(0).point.y;
		for (int i = 1; i < moPoints.size(); i++) {
			Point pt = moPoints.get(i).point;
			if (pt.x < minX) {
				minX = pt.x;
			}
			if (pt.x > maxX) {
				maxX = pt.x;
			}
			if (pt.y < minY) {
				minY = pt.y;
			}
			if (pt.y > maxY) {
				maxY = pt.y;
			}
		}

		double xRange = maxX - minX;
		double yRange = maxY - minY;
		if (xRange > MAX_SPREAD || xRange < MIN_SPREAD || yRange > MAX_SPREAD || yRange < MIN_SPREAD) {
			return false;
		} else {
			return true;
		}

	}

	public void registerMovingObjects2(Pose pose, List<Correspondence2D2D> prunedCorrespondences,
			List<MapPoint> prunedCorrespondenceMapPoints, List<Correspondence2D2D> outlierCorrespondences,
			List<MapPoint> outlierCorrespondenceMapPoints) {

		// determine window parameters
		double binSize = (Parameters.<Integer>get("cellSize")).doubleValue();
		int cellSearchRadius = 1;
		int margin = 4;

		// compress correspondences into matchedpoints
		List<Correspondence2D2D> allCorrespondences = new ArrayList<Correspondence2D2D>();
		List<MapPoint> allMapPoints = new ArrayList<MapPoint>();
		allCorrespondences.addAll(prunedCorrespondences);
		allMapPoints.addAll(prunedCorrespondenceMapPoints);
		allCorrespondences.addAll(outlierCorrespondences);
		allMapPoints.addAll(outlierCorrespondenceMapPoints);
		List<MatchedPoint> matchedPoints = this.buildMatchedPoints(allCorrespondences, allMapPoints);

		// load correspondences into hash table based on (x1, y1) position
		HashMap<String, List<MatchedPoint>> pointTable = this.buildPointTable(matchedPoints, binSize);
		HashMap<MatchedPoint, Boolean> potentiallyMoving = new HashMap<MatchedPoint, Boolean>();

		// slide window across table - for each group of correspondences...
		int numRows = (int) Math.ceil(Parameters.<Integer>get("height") / binSize);
		int numCols = (int) Math.ceil(Parameters.<Integer>get("width") / binSize);
		HashMap<String, Double> transChordals = new HashMap<String, Double>();
		HashMap<String, Double> rotChordals = new HashMap<String, Double>();
		for (int centerRow = margin; centerRow < numRows - margin; centerRow++) {
			for (int centerCol = margin; centerCol < numCols - margin; centerCol++) {

				transChordals.put(centerCol + "," + centerRow, 0.0);
				rotChordals.put(centerCol + "," + centerRow, 0.0);

				// cluster matchedPoints around this cell
				List<MatchedPoint> window = new ArrayList<MatchedPoint>();
				for (int row = centerRow - cellSearchRadius; row <= centerRow + cellSearchRadius; row++) {
					if (row < 0 || row >= numRows) {
						continue;
					}
					for (int col = centerCol - cellSearchRadius; col <= centerCol + cellSearchRadius; col++) {
						if (col < 0 || col >= numCols) {
							continue;
						}

						List<MatchedPoint> cell = pointTable.get(col + "," + row);
						if (cell != null) {
							window.addAll(cell);
						}

					}
				}

				if (window.size() < 6) {
					continue;
				}

				// prep for PnP
				List<Point3> point3s = new ArrayList<Point3>();
				List<Point> points = new ArrayList<Point>();
				List<Integer> inlierIndices = new ArrayList<Integer>();
				this.spreadMatchedPointList(window, point3s, points);

				// -- RANSAC PnP
				Matrix poseMat = new Matrix(4, 4);
				try {
					poseMat = Photogrammetry.OpenCVPnP(point3s, points, new Mat(), new Mat(), inlierIndices, false);
				} catch (Exception e) {
					continue;
				}

				// -- -- if there is a valid result check difference with estimated pose
				Matrix truePose = pose.getHomogeneousMatrix();
				double rotChordal = poseMat.getMatrix(0, 2, 0, 2).minus(truePose.getMatrix(0, 2, 0, 2)).normF();
				double transChordal = poseMat.getMatrix(0, 2, 3, 3).minus(truePose.getMatrix(0, 2, 3, 3)).normF();

//				Utils.pl("rotChordal: " + rotChordal);
//				Utils.pl("transChordal: " + transChordal);
				transChordals.put(centerCol + "," + centerRow, transChordal);
				rotChordals.put(centerCol + "," + centerRow, rotChordal);

				// -- -- -- if pose difference is large, register moving object for inliers
				if (rotChordal < 10 || transChordal < 10) {
					continue;
				}

				// add inliers to potentiallyMoving table
				for (Integer index : inlierIndices) {
					potentiallyMoving.put(window.get(index), true);
				}

			}
		}

		double transChordalHigh = 0;
		double rotChordalHigh = 0;
		for (int centerCol = margin; centerCol < numCols - margin; centerCol++) {
			for (int centerRow = margin; centerRow < numRows - margin; centerRow++) {
				String key = centerCol + "," + centerRow;
				double t = transChordals.get(key);
				double r = rotChordals.get(key);
				Utils.pl(String.format("transChordal[%s]: %.8f,        rotChordal[%s]: %.8f", key, t, key, r));
				if (t > transChordalHigh) {
					transChordalHigh = t;
				}
				if (r > rotChordalHigh) {
					rotChordalHigh = r;
				}
			}
		}

		Utils.pl("\n\nHIGHEST TRANSLATION CHORDAL: " + transChordalHigh);
		Utils.pl("HIGHEST ROTATION CHORDAL: " + rotChordalHigh);
		Utils.pl("\n\n");

		List<MatchedPoint> remainingPoints = new ArrayList<MatchedPoint>();
		List<MovingModel> movingObjects = new ArrayList<MovingModel>();
		List<MatchedPoint> potentiallyMovingList = new ArrayList<MatchedPoint>(potentiallyMoving.keySet());
		this.deduceMovingObjects2(potentiallyMovingList, pose, remainingPoints, movingObjects);

		Utils.pl("remainingPoints.size(): " + remainingPoints.size());
		Utils.pl("movingObjects.size(): " + movingObjects.size());

		this.map.registerMovingObjects(movingObjects);

	}

	public void deduceMovingObjects2(List<MatchedPoint> pointCorrespondences, Pose pose,
			List<MatchedPoint> outRemainingPoints, List<MovingModel> movingObjects) {

		outRemainingPoints.clear();
		outRemainingPoints.addAll(pointCorrespondences);

		movingObjects.clear();

		// init input for PnP
		List<Point3> point3s = new ArrayList<Point3>();
		List<Point> points = new ArrayList<Point>();
		List<Integer> inlierIndices = new ArrayList<Integer>();

		for (MatchedPoint pc : pointCorrespondences) {
			point3s.add(pc.point3);
			points.add(pc.point);
		}

		// loop to create moving models
		int MIN_POINTS = 6;
		boolean keepGoing = point3s.size() >= MIN_POINTS;
		while (keepGoing) {

			// model transformation matrix
			Matrix poseMat = new Matrix(4, 4);
			try {
				poseMat = Photogrammetry.OpenCVPnP(point3s, points, new Mat(), new Mat(), inlierIndices, false);
			} catch (Exception e) {
				keepGoing = false;
				Utils.pl("Moving object registration broken on localization (PnP)");
				continue;
			}

			// check if enough points in estimate
			if (inlierIndices.size() < MIN_POINTS) {
				keepGoing = false;
				continue;
			}

			// there are enough inliers for a model, construct model
			MovingModel mo = new MovingModel();
			Transformation poseReverse = (new Transformation(pose)).getReverseTransformation();
			mo.setTransformation(
					new Transformation(Utils.matrixToPose(poseReverse.getHomogeneousMatrix().times(poseMat))));

			// populate model with points
			HashMap<Integer, Boolean> isInlier = new HashMap<Integer, Boolean>();
			for (Integer idx : inlierIndices) {
				isInlier.put(idx, true);
			}

			// add inliers to MO and remove them from lists
			for (int i = outRemainingPoints.size() - 1; i >= 0; i--) {
				if (isInlier.get(i) == null) {
					continue;
				}
				mo.getMapPoints().add(outRemainingPoints.get(i).mapPoint);
				outRemainingPoints.remove(i);
				point3s.remove(i);
				points.remove(i);
			}

			movingObjects.add(mo);

			// if not enough outliers left, stop loop
			if (outRemainingPoints.size() < MIN_POINTS) {
				keepGoing = false;
			}

		}

	}

	public void spreadMatchedPointList(List<MatchedPoint> matchedPoints, List<Point3> outPoint3s,
			List<Point> outPoints) {
		outPoint3s.clear();
		outPoints.clear();
		for (MatchedPoint matchedPoint : matchedPoints) {
			outPoint3s.add(matchedPoint.point3);
			outPoints.add(matchedPoint.point);
		}
	}

	// reformat correspondences and map points into a single list with their
	// information
	// WARNING: MatchedPoints with no Point3D are thrown out, so resulting list size
	// may be different from input list sizes
	public List<MatchedPoint> buildMatchedPoints(List<Correspondence2D2D> correspondences, List<MapPoint> mapPoints) {
		List<MatchedPoint> matchedPoints = new ArrayList<MatchedPoint>();
		for (int i = 0; i < correspondences.size(); i++) {
			MatchedPoint matchedPoint = new MatchedPoint();
			matchedPoint.correspondence = correspondences.get(i);
			matchedPoint.mapPoint = mapPoints.get(i);
			matchedPoint.point3D = mapPoints.get(i).getPoint();
			if (matchedPoint.point3D == null) {
				continue;
			}
			matchedPoint.point3 = new Point3(matchedPoint.point3D.getX(), matchedPoint.point3D.getY(),
					matchedPoint.point3D.getZ());
			matchedPoint.point = new Point(matchedPoint.correspondence.getX1(), matchedPoint.correspondence.getY1());
			matchedPoints.add(matchedPoint);
		}
		return matchedPoints;
	}

	public HashMap<String, List<MatchedPoint>> buildPointTable(List<MatchedPoint> matchedPoints, double binLength) {
		HashMap<String, List<MatchedPoint>> pointTable = new HashMap<String, List<MatchedPoint>>();
		for (MatchedPoint matchedPoint : matchedPoints) {
			String key = this.getMatchedPointKey(matchedPoint, binLength);
			if (pointTable.get(key) == null) {
				pointTable.put(key, new ArrayList<MatchedPoint>());
			}
			pointTable.get(key).add(matchedPoint);
		}
		return pointTable;
	}

	public String getMatchedPointKey(MatchedPoint matchedPoint, double binLength) {
		int x = (int) (matchedPoint.correspondence.getX1() / binLength);
		int y = (int) (matchedPoint.correspondence.getY1() / binLength);
		return x + "," + y;
	}

	public void registerMovingObjects(Pose pose, List<Correspondence2D2D> prunedCorrespondences,
			List<MapPoint> prunedCorrespondenceMapPoints, List<Correspondence2D2D> outlierCorrespondences,
			List<MapPoint> outlierCorrespondenceMapPoints) {

		// reformat and combine correspondence data
		List<PointCorrespondence> prunedPCorr = new ArrayList<PointCorrespondence>();
		List<PointCorrespondence> outlierPCorr = new ArrayList<PointCorrespondence>();
		this.generatePointCorrespondences(prunedCorrespondences, prunedCorrespondenceMapPoints, false, prunedPCorr);
		this.generatePointCorrespondences(outlierCorrespondences, outlierCorrespondenceMapPoints, true, outlierPCorr);
		List<PointCorrespondence> allPCorr = new ArrayList<PointCorrespondence>();
		allPCorr.addAll(prunedPCorr);
		allPCorr.addAll(outlierPCorr);

		// identify high reprojection error points
		Matrix K = Parameters.getK();
		Matrix E = pose.getHomogeneousMatrix().getMatrix(0, 2, 0, 3);
		Matrix P = K.times(E);

		// -- get all projections and projection errors
		List<Double> projErrors = new ArrayList<Double>();
		for (PointCorrespondence pc : allPCorr) {
			Matrix proj = P.times(pc.point);
			double w = proj.get(2, 0);
			pc.projX = proj.get(0, 0) / w;
			pc.projY = proj.get(1, 0) / w;
			pc.reprojErr = Math.sqrt(Math.pow(pc.projX - pc.correspondence.getX1(), 2)
					+ Math.pow(pc.projY - pc.correspondence.getY1(), 2));
			projErrors.add(pc.reprojErr);
		}

		Dbl avg = new Dbl(0);
		Dbl stdDev = new Dbl(0);
		Dbl median = new Dbl(0);
		Utils.basicStats(projErrors, avg, stdDev, median);

		Utils.pl("avg: " + avg.getValue());
		Utils.pl("median: " + median.getValue());
		Utils.pl("avg / median: " + (avg.getValue() / median.getValue()));
		Utils.pl("stdDev: " + stdDev.getValue());
		Utils.pl("3 * stdDev: " + (3 * stdDev.getValue()));
		Utils.pl("median + 3 * stdDev: " + (median.getValue() + 3 * stdDev.getValue()));

//		String histogram = Utils.textHistogram(projErrors, 0.3, 120);
//		Utils.pl("Histogram: ");
//		Utils.pl(histogram);

		// -- isolate points that have more than mean + x * stdDev reprojection error
		List<PointCorrespondence> potentiallyMoving = new ArrayList<PointCorrespondence>();
		if (avg.getValue() / median.getValue() > 50) {
			for (PointCorrespondence pc : allPCorr) {
				if (pc.reprojErr > avg.getValue()) {
					potentiallyMoving.add(pc);
				}
			}
		}

		Utils.pl("\n\npotentiallyMoving.size(): " + potentiallyMoving.size() + "\n\n");

		List<PointCorrespondence> remainingPoints = new ArrayList<PointCorrespondence>();
		List<MovingModel> movingObjects = new ArrayList<MovingModel>();
		this.deduceMovingObjects(potentiallyMoving, pose, remainingPoints, movingObjects);

		Utils.pl("remainingPoints.size(): " + remainingPoints.size());
		Utils.pl("movingObjects.size(): " + movingObjects.size());

		this.map.registerMovingObjects(movingObjects);

	}

	public void deduceMovingObjects(List<PointCorrespondence> pointCorrespondences, Pose pose,
			List<PointCorrespondence> outRemainingPoints, List<MovingModel> movingObjects) {

		outRemainingPoints.clear();
		outRemainingPoints.addAll(pointCorrespondences);

		movingObjects.clear();

		// init input for PnP
		List<Point3> point3s = new ArrayList<Point3>();
		List<Point> points = new ArrayList<Point>();
		List<Integer> inlierIndices = new ArrayList<Integer>();

		for (PointCorrespondence pc : pointCorrespondences) {
			Point3 pt3 = new Point3(pc.point3D.getX(), pc.point3D.getY(), pc.point3D.getZ());
			Point pt = new Point(pc.correspondence.getX1(), pc.correspondence.getY1());
			point3s.add(pt3);
			points.add(pt);
		}

		// loop to create moving models
		int MIN_POINTS = 6;
		boolean keepGoing = point3s.size() >= MIN_POINTS;
		while (keepGoing) {

			// model transformation matrix
			Matrix poseMat = new Matrix(4, 4);
			try {
				poseMat = Photogrammetry.OpenCVPnP(point3s, points, new Mat(), new Mat(), inlierIndices, false);
			} catch (Exception e) {
				keepGoing = false;
				Utils.pl("Moving object registration broken on localization (PnP)");
				continue;
			}

			// check if enough points in estimate
			if (inlierIndices.size() < MIN_POINTS) {
				keepGoing = false;
				continue;
			}

			// there are enough inliers for a model, construct model
			MovingModel mo = new MovingModel();
			Transformation poseReverse = (new Transformation(pose)).getReverseTransformation();
			mo.setTransformation(
					new Transformation(Utils.matrixToPose(poseReverse.getHomogeneousMatrix().times(poseMat))));

			// populate model with points
			HashMap<Integer, Boolean> isInlier = new HashMap<Integer, Boolean>();
			for (Integer idx : inlierIndices) {
				isInlier.put(idx, true);
			}

			// add inliers to MO and remove them from lists
			for (int i = outRemainingPoints.size() - 1; i >= 0; i--) {
				if (isInlier.get(i) == null) {
					continue;
				}
				mo.getMapPoints().add(outRemainingPoints.get(i).mapPoint);
				outRemainingPoints.remove(i);
				point3s.remove(i);
				points.remove(i);
			}

			movingObjects.add(mo);

			// if not enough outliers left, stop loop
			if (outRemainingPoints.size() < MIN_POINTS) {
				keepGoing = false;
			}

		}

	}

	// find moving objects in frame and update their poses
	public void trackMovingObjects(Pose pose, MatOfKeyPoint keypoints, Mat descriptors) {

		List<KeyPoint> keypointsList = keypoints.toList();

		// get all moving objects
		List<MovingModel> movingObjects = this.map.getMovingObjects();

		// for each moving object,
		for (MovingModel mo : movingObjects) {

			// -- get its descriptors
			List<Mat> moDescriptors = new ArrayList<Mat>();
			List<MapPoint> descriptorMapPoints = new ArrayList<MapPoint>();
			for (MapPoint mp : mo.getMapPoints()) {

				Observation o = mp.getPrincipleObservation();

				if (o == null) {
					continue;
				}

				Keyframe kf = o.getKeyframe();
				List<Integer> mpIndices = kf.getMapPointIndices().get(mp);
				for (Integer mpIdx : mpIndices) {
					moDescriptors.add(kf.getDescriptorsList().get(mpIdx));
					descriptorMapPoints.add(mp);
				}

			}

			// -- feature match the descriptors to the current descriptors
			List<DMatch> matches = ORBMatcher.matchDescriptors(descriptors, Utils.mergeDescriptorList(moDescriptors));

			// -- if >= 6 matches, attempt PnP estimation
			if (matches.size() < 6) {
				continue;
			}

			// create PnP input
			List<Point3> p3s = new ArrayList<Point3>();
			List<Point> p2s = new ArrayList<Point>();
			for (DMatch match : matches) {
				Point3D point3D = descriptorMapPoints.get(match.queryIdx).getPoint();
				if (point3D == null) {
					continue;
				}
				Point3 p3 = new Point3(point3D.getX(), point3D.getY(), point3D.getZ());
				Point p2 = keypointsList.get(match.trainIdx).pt;
				p3s.add(p3);
				p2s.add(p2);
			}

			if (p3s.size() < 6) {
				continue;
			}

			// -- -- PnP estimation
			List<Integer> inlierIndices = new ArrayList<Integer>();
			Matrix E = null;
			try {
				Mat rvec = new Mat();
				Mat tvec = new Mat();
//				if (mo.pose != null) {
//					Utils.poseToRodrigues(pose, rvec, tvec);
//				}
				E = Photogrammetry.OpenCVPnP(p3s, p2s, rvec, tvec, inlierIndices, false, true);
			} catch (Exception e) {
//				e.printStackTrace();
			}

			// -- -- if PnP estimation works, convert to correct pose and save in MO
			if (E == null) {
				continue;
			}

			Transformation poseReverse = (new Transformation(pose)).getReverseTransformation();
			Transformation objectPose = new Transformation(
					Utils.matrixToPose(poseReverse.getHomogeneousMatrix().times(E)));

			boolean inFrontOfCamera = this.isInFrontOfCamera(pose, objectPose, mo);
			double distFromCamera = objectPose.getDistanceFrom(pose);

			// check if pose is far from camera
//			if (distFromCamera > 100) {
//				continue;
//			}

			// check if behind camera
			if (!inFrontOfCamera) {
//				Utils.pl("BEHIND CAMERA");
				continue;
			}

			// check reprojection error
			List<Double> reprojErrors = this.getReprojectionErrors(p3s, p2s, pose, objectPose);
			Dbl avg = new Dbl(0);
			Dbl stdDev = new Dbl(0);
			Dbl median = new Dbl(0);
			Utils.basicStats(reprojErrors, avg, stdDev, median);

			if (avg.getValue() > 4) {
				continue;
			}

			mo.setTransformation(objectPose);

		}

	}

	public List<Double> getReprojectionErrors(List<Point3> p3s, List<Point> p2s, Pose pose, Transformation objectPose) {
		List<Double> reprojectionErrors = new ArrayList<Double>();

		// prep matrices
		Matrix cameraPose = pose.getHomogeneousMatrix();
		Matrix objPose = objectPose.getHomogeneousMatrix();
		Matrix E = cameraPose.times(objPose);

		// load data into matrix
		Matrix points = new Matrix(4, p3s.size(), 1);
		for (int i = 0; i < p3s.size(); i++) {
			Point3 p = p3s.get(i);
			points.set(0, i, p.x);
			points.set(1, i, p.y);
			points.set(2, i, p.z);
		}

		// get projections
		Matrix projections = Parameters.getK4x4().times(E).times(points);

		// get errors
		for (int i = 0; i < p2s.size(); i++) {
			Point p = p2s.get(i);
			double w = projections.get(2, i);
			double x = projections.get(0, i) / w;
			double y = projections.get(1, i) / w;
			double error = Math.sqrt(Math.pow(x - p.x, 2) + Math.pow(y - p.y, 2));
			reprojectionErrors.add(error);
		}

		return reprojectionErrors;
	}

	public boolean isInFrontOfCamera(Pose pose, Transformation objectPose, MovingModel movingObject) {

		Matrix E = pose.getHomogeneousMatrix().times(objectPose.getHomogeneousMatrix());
		List<Point3D> points = new ArrayList<Point3D>();

		// collect points
		for (MapPoint mp : movingObject.getMapPoints()) {
			Point3D p = mp.getPoint();
			if (p != null) {
				points.add(p);
			}
		}

		if (points.size() <= 0) {
			return false;
		}

		// format points into matrix
		Matrix pointMatrix = new Matrix(4, points.size(), 1);
		for (int i = 0; i < points.size(); i++) {
			Point3D p = points.get(i);
			pointMatrix.set(0, i, p.getX());
			pointMatrix.set(1, i, p.getY());
			pointMatrix.set(2, i, p.getZ());
		}

		// transform points
		Matrix transformedPoints = E.times(pointMatrix);

		// tally points in front of camera
		double inFront = 0;
		for (int i = 0; i < transformedPoints.getColumnDimension(); i++) {
			if (transformedPoints.get(2, i) > 0) {
				inFront++;
			}
		}

		return (inFront / points.size() > 0.5);

	}

	public int trackMovement(MatOfKeyPoint keypoints, Mat descriptors, List<Correspondence2D2D> outCorrespondences,
			List<Correspondence2D2D> outPrunedCorrespondences, List<MapPoint> outPrunedCorrespondenceMapPoints,
			List<Correspondence2D2D> outOutlierCorrespondences, List<MapPoint> outOutlierCorrespondenceMapPoints,
			List<MapPoint> outMapPointPerDescriptor, List<Correspondence2D2D> outUntriangulatedCorrespondences,
			List<MapPoint> outUntriangulatedMapPoints, Pose outPose, Dbl outInlierRate, Dbl outNumTracked)
			throws TrackingException {
		return this.trackMovementFromKeyframe(this.map.getCurrentKeyframe(), keypoints, descriptors, outCorrespondences,
				outPrunedCorrespondences, outPrunedCorrespondenceMapPoints, outOutlierCorrespondences,
				outOutlierCorrespondenceMapPoints, outMapPointPerDescriptor, outUntriangulatedCorrespondences,
				outUntriangulatedMapPoints, outPose, outInlierRate, outNumTracked);
	}

	// does full frame feature matching, generates correspondences, gets PnP pose,
	// and returns the number of matches
	public int trackMovementFromKeyframe(Keyframe keyframe, MatOfKeyPoint keypoints, Mat descriptors,
			List<Correspondence2D2D> outCorrespondences, List<Correspondence2D2D> outPrunedCorrespondences,
			List<MapPoint> outPrunedCorrespondenceMapPoints, List<Correspondence2D2D> outOutlierCorrespondences,
			List<MapPoint> outOutlierCorrespondenceMapPoints, List<MapPoint> outMapPointPerDescriptor,
			List<Correspondence2D2D> outUntriangulatedCorrespondences, List<MapPoint> outUntriangulatedMapPoints,
			Pose outPose, Dbl outInlierRate, Dbl outNumTracked) throws TrackingException {

		outCorrespondences.clear();
		outMapPointPerDescriptor.clear();
		outUntriangulatedCorrespondences.clear();
		outUntriangulatedMapPoints.clear();

		List<KeyPoint> keypointList = keypoints.toList();
		List<KeyPoint> keyframeKeypointList = keyframe.getKeypoints().toList();

		for (int i = 0; i < descriptors.rows(); i++) {
			outMapPointPerDescriptor.add(null);
		}

		List<DMatch> matches = ORBMatcher.matchDescriptors(keyframe, descriptors);
		Utils.pl("number of features: " + descriptors.rows());
		Utils.pl("number of matches: " + matches.size());

		// generate correspondences, map point list, and input for PnP
		int numTracked = 0;
		List<Point3> point3s = new ArrayList<Point3>();
		List<Point> points = new ArrayList<Point>();
		for (int i = 0; i < matches.size(); i++) {

			// add correspondence
			Correspondence2D2D c = new Correspondence2D2D();
			c.setX0(keyframeKeypointList.get(matches.get(i).trainIdx).pt.x);
			c.setY0(keyframeKeypointList.get(matches.get(i).trainIdx).pt.y);
			c.setX1(keypointList.get(matches.get(i).queryIdx).pt.x);
			c.setY1(keypointList.get(matches.get(i).queryIdx).pt.y);
			c.computeLength();
			outCorrespondences.add(c);
		}

		ORBMatcher.blindPrune(outCorrespondences, matches);

		List<Mat> descriptorsList = Utils.descriptorList(descriptors);
		for (int i = 0; i < matches.size(); i++) {

			Correspondence2D2D c = outCorrespondences.get(i);

			// update keyframe descriptor
			keyframe.getMostRecentDescriptorsList().set(matches.get(i).trainIdx,
					descriptorsList.get(matches.get(i).queryIdx));

			// set map point in list
			MapPoint mp = keyframe.getMapPoints().get(matches.get(i).trainIdx);
			outMapPointPerDescriptor.set(matches.get(i).queryIdx, mp);

			// if it is already triangulated, extract point data for PnP
			if (mp.getPoint() == null) {
				outUntriangulatedCorrespondences.add(c);
				outUntriangulatedMapPoints.add(mp);
				continue;
			}

			numTracked++;

			// if the point is tied to a moving object, do not include in PnP estimate
			if (mp.getAssociatedObject() != null) {
				continue;
			}

			outPrunedCorrespondences.add(c);
			outPrunedCorrespondenceMapPoints.add(mp);

			Point3 point3 = new Point3(mp.getPoint().getX(), mp.getPoint().getY(), mp.getPoint().getZ());
			point3s.add(point3);
			Point point = keypointList.get(matches.get(i).queryIdx).pt;
			points.add(point);

		}

		outNumTracked.setValue(numTracked);
		Utils.pl("numTracked = " + numTracked);

		List<Integer> inlierIndices = new ArrayList<Integer>();
		long start = System.currentTimeMillis();
		Matrix E = Photogrammetry.OpenCVPnP(point3s, points, new Mat(), new Mat(), inlierIndices, true);
		long end = System.currentTimeMillis();
		Utils.pl("PnP time: " + (end - start) + "ms");

		// set inlier rate (inliers / numTracked)
		outInlierRate.setValue((double) inlierIndices.size() / numTracked);

		outPose.setPose(Utils.matrixToPose(E));

		// prune outliers
		int numPruned = 0;
		Utils.pl("inlierIndices.size(): " + inlierIndices.size());

		HashMap<Integer, Boolean> isInlier = new HashMap<Integer, Boolean>();
		for (int i = 0; i < inlierIndices.size(); i++) {
			isInlier.put(inlierIndices.get(i), true);
		}
		for (int i = outPrunedCorrespondences.size() - 1; i >= 0; i--) {
			// if inlier, skip
			if (isInlier.get(i) != null) {
				continue;
			}
			// if outlier, remove
			numPruned++;
			outOutlierCorrespondences.add(outPrunedCorrespondences.get(i));
			outOutlierCorrespondenceMapPoints.add(outPrunedCorrespondenceMapPoints.get(i));
			outPrunedCorrespondences.remove(i);
			MapPoint mp = outPrunedCorrespondenceMapPoints.get(i);
			if (outMapPointPerDescriptor.indexOf(mp) != -1) {
				outMapPointPerDescriptor.set(outMapPointPerDescriptor.indexOf(mp), null);
			}
			outPrunedCorrespondenceMapPoints.remove(i);
		}

		Utils.pl("pruned correspondences size: " + outPrunedCorrespondences.size());

		return matches.size();

	}

	public void partitionedPointInclusion(MatOfKeyPoint keypoints, Mat descriptors,
			List<MapPoint> mapPointPerDescriptor, List<Correspondence2D2D> outPrunedCorrespondences,
			List<MapPoint> outPrunedCorrespondenceMapPoints, Pose outPose) {

		// gather map points in neighboring partitions to consider for tracking
		List<MapPoint> neighboringMapPoints = new ArrayList<MapPoint>();
		gatherNeighboringMapPoints(mapPointPerDescriptor, 1, neighboringMapPoints);

		int mergedMPs = 0;
		for (int i = 0; i < mapPointPerDescriptor.size(); i++) {
			if (mapPointPerDescriptor.get(i) != null) {
				mergedMPs += mapPointPerDescriptor.get(i).getMergedTo() != null ? 1 : 0;
			}
		}

		// get projections
		List<ProjectionInfo> projectionInfos = new ArrayList<ProjectionInfo>();
		loadProjections(neighboringMapPoints, outPose, projectionInfos);

		// prune off-screen points
		pruneOffScreenPoints(projectionInfos);

		// gather descriptors for map points
		List<DescriptorInfo> descriptorInfos = new ArrayList<DescriptorInfo>();
		gatherDescriptors(projectionInfos, outPose, descriptorInfos);

		// spacially filter descriptors to keep size down
		spaciallyFilterDescriptors(descriptorInfos, 40, 5, descriptorInfos);

		// guided matching
		List<KeyPoint> keypointsList = keypoints.toList();
		List<Mat> descriptorsList = Utils.descriptorList(descriptors);
		List<DMatch> matches = ORBMatcher.guidedMatching(descriptorInfos, descriptorsList, keypointsList, 100);

		// catalog tracked map points
		HashMap<MapPoint, Boolean> mapPointsTracked = new HashMap<MapPoint, Boolean>();
		for (DMatch match : matches) {
			mapPointsTracked.put(descriptorInfos.get(match.queryIdx).projectionInfo.mapPoint, true);
		}

		// PnP tracking
		List<Integer> inlierIndices = new ArrayList<Integer>();
		PnPTracking(descriptorInfos, keypointsList, matches, outPose, inlierIndices);

		// update map point per descriptor with new matches
		outPrunedCorrespondences.clear();
		outPrunedCorrespondenceMapPoints.clear();
		for (int i = 0; i < inlierIndices.size(); i++) {

			DMatch match = matches.get(inlierIndices.get(i));

			// add correspondence
			Correspondence2D2D c = new Correspondence2D2D();
			c.setX0(descriptorInfos.get(match.queryIdx).keypoint.pt.x);
			c.setY0(descriptorInfos.get(match.queryIdx).keypoint.pt.y);
			c.setX1(keypointsList.get(match.trainIdx).pt.x);
			c.setY1(keypointsList.get(match.trainIdx).pt.y);
			c.computeLength();
			outPrunedCorrespondences.add(c);

			mapPointPerDescriptor.set(match.trainIdx, descriptorInfos.get(match.queryIdx).projectionInfo.mapPoint);
			outPrunedCorrespondenceMapPoints.add(descriptorInfos.get(match.queryIdx).projectionInfo.mapPoint);
		}

		// record points that were tracked and points that failed
		for (DescriptorInfo di : descriptorInfos) {
			MapPoint mp = di.projectionInfo.mapPoint;
			Boolean tracked = mapPointsTracked.get(mp);
			if (tracked != null) {
				mp.incTimesTracked();
			} else {
				mp.incTimesLost();
			}
		}

	}

	// given a list of map points (mapPoints), return a list
	// (outNeighboringMapPoints) including those map points and map points in
	// partitions that are within the search radius (radius) of any map point in the
	// map point list
	public void gatherNeighboringMapPoints(List<MapPoint> mapPoints, int radius,
			List<MapPoint> outNeighboringMapPoints) {

		// gather all map points in partitions included by given map points
		HashMap<String, List<MapPoint>> partitions = new HashMap<String, List<MapPoint>>();
		for (int i = 0; i < mapPoints.size(); i++) {
			MapPoint mp = mapPoints.get(i);
			if (mp == null || mp.getPoint() == null) {
				continue;
			}
			String partitionKey = this.map.getPartitionKey(mp.getPoint());
			// get all neighboring partitions
			String[] splits = partitionKey.split(",");
			int x = Integer.parseInt(splits[0]);
			int y = Integer.parseInt(splits[1]);
			int z = Integer.parseInt(splits[2]);
			for (int dx = -radius; dx <= radius; dx++) {
				for (int dy = -radius; dy <= radius; dy++) {
					for (int dz = -radius; dz <= radius; dz++) {
						String key = (x + dx) + "," + (y + dy) + "," + (z + dz);
						if (partitions.get(key) != null) {
							continue;
						}
						List<MapPoint> partition = this.map.getMapPointsByPartition().get(key);
						if (partition != null) {
							partitions.put(key, partition);
						}
					}
				}
			}
		}

		// flatten partitions into output list
		outNeighboringMapPoints.clear();
		for (String key : partitions.keySet()) {
			outNeighboringMapPoints.addAll(partitions.get(key));
		}

		// filter out points that have been merged (this is sort of a band-aid)
		outNeighboringMapPoints.stream().filter(mp -> mp.getMergedTo() == null).collect(Collectors.toList());

	}

	// given a list of map points (mapPoints) and a current estimation of a camera
	// pose (pose), return a list of the projection info (outProjectionInfos) for
	// each map point, pruning out map points that are missing a point3D (prune both
	// from mapPoints and outProjectionInfos)
	public void loadProjections(List<MapPoint> mapPoints, Pose pose, List<ProjectionInfo> outProjectionInfos) {

		// initialize projectionInfos with map points and point3Ds
		outProjectionInfos.clear();
		for (int i = 0; i < mapPoints.size(); i++) {
			MapPoint mp = mapPoints.get(i);
			ProjectionInfo pi = new ProjectionInfo();
			pi.mapPoint = mp;
			pi.point3D = mp.getPoint();
			if (pi.point3D == null) {
				mapPoints.remove(i);
				i--;
				continue;
			}
			outProjectionInfos.add(pi);
		}

		// get projections
		Matrix P = Parameters.getK().times(pose.getHomogeneousMatrix().getMatrix(0, 2, 0, 3));
		Matrix X = new Matrix(4, outProjectionInfos.size(), 1);
		for (int i = 0; i < outProjectionInfos.size(); i++) {
			Point3D p = outProjectionInfos.get(i).point3D;
			X.set(0, i, p.getX());
			X.set(1, i, p.getY());
			X.set(2, i, p.getZ());
			// 4th row already set to 1 in initialization
		}
		Matrix x = P.times(X);

		// load projection data into projectionInfos
		for (int i = 0; i < outProjectionInfos.size(); i++) {
			ProjectionInfo pi = outProjectionInfos.get(i);
			double wValue = x.get(2, i);
			pi.wValue = wValue;
			Point pt = new Point(x.get(0, i) / wValue, x.get(1, i) / wValue);
			pi.projection = pt;
		}

	}

	// given a list of map point projection information (projectionInfos), remove
	// entries that are off-screen
	public void pruneOffScreenPoints(List<ProjectionInfo> projectionInfos) {

		for (int i = projectionInfos.size() - 1; i >= 0; i--) {
			double w = projectionInfos.get(i).wValue;
			Point proj = projectionInfos.get(i).projection;
			if (w < 0 || proj.x < 0 || proj.y < 0 || proj.x > Parameters.<Integer>get("width")
					|| proj.y > Parameters.<Integer>get("height")) {
				projectionInfos.remove(i);
			}
		}

	}

	// given a list of map point projection information (projectionInfos) and an
	// estimation of the camera pose (pose), return a list of each descriptor
	// associated with the priciple observation of each map point with their
	// associated keypoint and projection info (outDescriptorInfos)
	public void gatherDescriptors(List<ProjectionInfo> projectionInfos, Pose pose,
			List<DescriptorInfo> outDescriptorInfos) {

		outDescriptorInfos.clear();
		for (ProjectionInfo pi : projectionInfos) {

			// skip if no principle observation
			Observation po = pi.mapPoint.getPrincipleObservation();
			if (po == null) {
				continue;
			}

			// calculate expected normal
			Point3D point = pi.point3D;
			Matrix normal = new Matrix(3, 1);
			normal.set(0, 0, point.getX() - pose.getCx());
			normal.set(1, 0, point.getY() - pose.getCy());
			normal.set(2, 0, point.getZ() - pose.getCz());
			normal = normal.times(1 / normal.normF());

			// get priciple keyframe and its normal
			Keyframe bestKF = po.getKeyframe();
			if (po.getNormal() == null) {
				po.calculateNormal(pi.mapPoint);
			}
			Matrix bestNorm = po.getNormal();

			// check angle between normals. if it exceeds some amount, ignore this map point
			double dot = normal.get(0, 0) * bestNorm.get(0, 0) + normal.get(1, 0) * bestNorm.get(1, 0)
					+ normal.get(2, 0) * bestNorm.get(2, 0);

			if (dot < 0.49998) { // cos(60 degrees)
				continue;
			}

			// load descriptor infos with info on each descriptor for this observation
			List<Integer> descriptorIndices = bestKF.getMapPointIndices().get(pi.mapPoint);
			if (descriptorIndices == null) { // bug?
				continue;
			}
			for (int j = 0; j < descriptorIndices.size(); j++) {
				int index = descriptorIndices.get(j);
				KeyPoint keypoint = bestKF.getKeypointsList().get(index);
				Mat descriptor = bestKF.getMostRecentDescriptorsList().get(index);
				DescriptorInfo di = new DescriptorInfo();
				di.descriptor = descriptor;
				di.keypoint = keypoint;
				di.projectionInfo = pi;
				outDescriptorInfos.add(di);
			}

		}

	}

	// limit number of descriptors (descriptorInfos) by performing a
	// pseudo-non-maximum suppression where points with higher tracking instances
	// are prioritized. numColumns represents the number of columns the screen is
	// divided into and the resulting cell size (height and width) are both
	// determined by this value. numPerBin determines the number of descriptors that
	// will be kept in each bin. the remaining descriptors go into
	// outRemainingDescriptorInfos
	public void spaciallyFilterDescriptors(List<DescriptorInfo> descriptorInfos, int numColumns, int numPerBin,
			List<DescriptorInfo> outRemainingDescriptorInfos) {

		// create spacial table
		int CELL_SIZE = Parameters.<Integer>get("width") / numColumns;
		HashMap<String, List<DescriptorInfo>> descriptorTable = new HashMap<String, List<DescriptorInfo>>();
		for (DescriptorInfo di : descriptorInfos) {
			int x = (int) (di.projectionInfo.projection.x / CELL_SIZE);
			int y = (int) (di.projectionInfo.projection.y / CELL_SIZE);
			String key = x + "," + y;
			if (descriptorTable.get(key) == null) {
				descriptorTable.put(key, new ArrayList<DescriptorInfo>());
			}
			descriptorTable.get(key).add(di);
		}

		// sort each partition
		for (String key : descriptorTable.keySet()) {
			List<DescriptorInfo> diList = descriptorTable.get(key);
			Collections.sort(diList, new Comparator<DescriptorInfo>() {
				public int compare(DescriptorInfo di1, DescriptorInfo di2) {
					int diff = (int) (di1.projectionInfo.mapPoint.timesTracked
							- di2.projectionInfo.mapPoint.timesTracked);
					return diff;
				}
			});

		}

		// add filtered number of descriptors to final list
		List<DescriptorInfo> finalDIList = new ArrayList<DescriptorInfo>();
		for (String key : descriptorTable.keySet()) {
			List<DescriptorInfo> diList = descriptorTable.get(key);
			if (diList.size() <= numPerBin) {
				finalDIList.addAll(diList);
			} else {
				finalDIList.addAll(diList.subList(0, numPerBin));
			}
		}

		// set output list
		outRemainingDescriptorInfos.clear();
		outRemainingDescriptorInfos.addAll(finalDIList);

	}

	// given a list of descriptor info (descriptorInfos), a list of observed
	// keypoints (keypointsList), and the matches between them (matches, where
	// queryIdx -> descriptorInfos and trainIdx -> keypointsList), re-estimate the
	// pose (outPose) with PnP and log the indices of the inliers (inlierIndices,
	// where each index is related to an entry in matches)
	public void PnPTracking(List<DescriptorInfo> descriptorInfos, List<KeyPoint> keypointsList, List<DMatch> matches,
			Pose outPose, List<Integer> outInlierIndices) {

		// create input for PnP
		outInlierIndices.clear();
		List<Point3> point3s = new ArrayList<Point3>();
		List<Point> points = new ArrayList<Point>();
		for (int i = 0; i < matches.size(); i++) {
			DMatch match = matches.get(i);
			Point3D point3D = descriptorInfos.get(match.queryIdx).projectionInfo.point3D;
			Point3 p3 = new Point3(point3D.getX(), point3D.getY(), point3D.getZ());
			point3s.add(p3);
			points.add(new Point(keypointsList.get(match.trainIdx).pt.x, keypointsList.get(match.trainIdx).pt.y));
		}

		Matrix E = Photogrammetry.OpenCVPnP(point3s, points, new Mat(), new Mat(), outInlierIndices, true);
		outPose.setPose(Utils.matrixToPose(E));
		Utils.pl("inliers (after point inclusion): " + outInlierIndices.size());

	}

	public Map getMap() {
		return map;
	}

	public void setMap(Map map) {
		this.map = map;
	}

}
