package lumoslam;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.ejml.data.DMatrixRMaj;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;

import Jama.Matrix;
import Jama.SingularValueDecomposition;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.so.Quaternion_F64;
import runtimevars.Parameters;
import toolbox.Utils;
import types.Correspondence2D2D;
import types.Dbl;
import types.Point3D;
import types.Pose;

public class Photogrammetry {

	public static void getCheiralityMap(List<MapPoint> mapPointPerDescriptor, Pose pose,
			HashMap<MapPoint, Double> cheiralityValues) {

		List<MapPoint> triangulatedMapPoints = mapPointPerDescriptor.stream()
				.filter(mp -> mp != null && mp.getPoint() != null).collect(Collectors.toList());

		Matrix X = new Matrix(4, triangulatedMapPoints.size(), 1);

		// load points
		for (int i = 0; i < triangulatedMapPoints.size(); i++) {
			Point3D pt = triangulatedMapPoints.get(i).getPoint();
			if (pt == null) {
				pt = new Point3D(pose.getCx(), pose.getCy(), pose.getCz());
			}
			X.set(0, i, pt.getX());
			X.set(1, i, pt.getY());
			X.set(2, i, pt.getZ());
		}

		// transform points
		Matrix transformed = pose.getHomogeneousMatrix().times(X);

		// evaluate Z values for cheirality
		for (int i = 0; i < transformed.getColumnDimension(); i++) {
			cheiralityValues.put(triangulatedMapPoints.get(i), transformed.get(2, i));
		}

	}

	public static Pose SFMFundamentalMatrixEstimate(List<Correspondence2D2D> correspondences, List<Boolean> inlierMap,
			Dbl numWithHighParallax, Dbl goodPoints) {

		double ransacReprojThreshold = 0.25;

		double parallaxRequirement = Parameters.<Double>get("parallaxRequirement");
		double initializationReprojectionError = Parameters.<Double>get("initializationReprojectionError");

		// create point matrices
		List<Point> points0 = new ArrayList<Point>();
		List<Point> points1 = new ArrayList<Point>();
		for (int i = 0; i < correspondences.size(); i++) {
			Point point0 = new Point(correspondences.get(i).getX0(), correspondences.get(i).getY0());
			Point point1 = new Point(correspondences.get(i).getX1(), correspondences.get(i).getY1());
			points0.add(point0);
			points1.add(point1);
		}

		MatOfPoint2f points0Mat = new MatOfPoint2f();
		MatOfPoint2f points1Mat = new MatOfPoint2f();
		points0Mat.fromList(points0);
		points1Mat.fromList(points1);

		long start = System.currentTimeMillis();
//		Mat fundamentalMatrix = Calib3d.findFundamentalMat(points0Mat, points1Mat, Calib3d.FM_7POINT);
		Mat fundamentalMatrix = Calib3d.findFundamentalMat(points0Mat, points1Mat, Calib3d.FM_RANSAC,
				ransacReprojThreshold, 0.99, 500);
		long end = System.currentTimeMillis();
		Utils.pl("Fundamental matrix estimation time: " + (end - start) + "ms");

		inlierMap.clear();
		Matrix F = Utils.MatToMatrix(fundamentalMatrix);
		int numInliers = 0;
		for (int i = 0; i < correspondences.size(); i++) {
			Matrix p0 = new Matrix(3, 1);
			Matrix p1 = new Matrix(3, 1);
			p0.set(0, 0, correspondences.get(i).getX0());
			p0.set(1, 0, correspondences.get(i).getY0());
			p0.set(2, 0, 1);
			p1.set(0, 0, correspondences.get(i).getX1());
			p1.set(1, 0, correspondences.get(i).getY1());
			p1.set(2, 0, 1);
			double constraintVal = Math.abs(p1.transpose().times(F).times(p0).get(0, 0));
//			Utils.pl("constraintVal: " + constraintVal);
			inlierMap.add(constraintVal < ransacReprojThreshold);
			numInliers = constraintVal < ransacReprojThreshold ? numInliers + 1 : numInliers;
		}

		Utils.pl("inliers: " + numInliers + " / " + correspondences.size());

		// convert to essential matrix
		Mat K = Parameters.getKMat();
		Mat Kt = new Mat();
		Core.transpose(K, Kt);
		Mat E = new Mat();
		Core.gemm(Kt, fundamentalMatrix, 1, new Mat(), 0, E, 0);
		Core.gemm(E, K, 1, new Mat(), 0, E, 0);

		// decompose essential matrix
		Mat R1Mat = new Mat();
		Mat R2Mat = new Mat();
		Mat tMat = new Mat();
		Calib3d.decomposeEssentialMat(E, R1Mat, R2Mat, tMat);

		Matrix R1 = Utils.MatToMatrix(R1Mat);
		Matrix R2 = Utils.MatToMatrix(R2Mat);
		Matrix t = Utils.MatToMatrix(tMat);

		// triangulate point and select correct solution (cheirality)
		Matrix I = Matrix.identity(4, 4);
		Matrix R1t1 = Matrix.identity(4, 4);
		Matrix R1t2 = Matrix.identity(4, 4);
		Matrix R2t1 = Matrix.identity(4, 4);
		Matrix R2t2 = Matrix.identity(4, 4);
		Matrix[] possiblePoses = { R1t1, R1t2, R2t1, R2t2 };

		R1t1.setMatrix(0, 2, 0, 2, R1);
		R1t1.setMatrix(0, 2, 3, 3, t);
		R1t2.setMatrix(0, 2, 0, 2, R1);
		R1t2.setMatrix(0, 2, 3, 3, t.times(-1));
		R2t1.setMatrix(0, 2, 0, 2, R2);
		R2t1.setMatrix(0, 2, 3, 3, t);
		R2t2.setMatrix(0, 2, 0, 2, R2);
		R2t2.setMatrix(0, 2, 3, 3, t.times(-1));

		Random rand = new Random();
		int[] scores = { 0, 0, 0, 0 };

		// additionally, record the number of points that triangulated with sufficient
		// parallax for each hypothesis
		int[] numHighParallax = { 0, 0, 0, 0 };

		for (int i = 0; i < correspondences.size(); i++) {

			if (!inlierMap.get(i)) {
				continue;
			}

			Correspondence2D2D c = correspondences.get(i);

			// record parallax of the points
			Dbl parallaxR1t1 = new Dbl(0);
			Dbl parallaxR1t2 = new Dbl(0);
			Dbl parallaxR2t1 = new Dbl(0);
			Dbl parallaxR2t2 = new Dbl(0);

			// reprojection errors
			Dbl err0R1t1 = new Dbl(0);
			Dbl err1R1t1 = new Dbl(0);
			Dbl err0R1t2 = new Dbl(0);
			Dbl err1R1t2 = new Dbl(0);
			Dbl err0R2t1 = new Dbl(0);
			Dbl err1R2t1 = new Dbl(0);
			Dbl err0R2t2 = new Dbl(0);
			Dbl err1R2t2 = new Dbl(0);

			// w values of reprojections (for cheirality check)
			Dbl wR1t1 = new Dbl(0);
			Dbl wR1t2 = new Dbl(0);
			Dbl wR2t1 = new Dbl(0);
			Dbl wR2t2 = new Dbl(0);

			// get triangulated 3D points
			Matrix point3DR1t1 = triangulate(R1t1, I, c, parallaxR1t1, err1R1t1, err0R1t1, wR1t1, new Dbl(0));
			Matrix point3DR1t2 = triangulate(R1t2, I, c, parallaxR1t2, err1R1t2, err0R1t2, wR1t2, new Dbl(0));
			Matrix point3DR2t1 = triangulate(R2t1, I, c, parallaxR2t1, err1R2t1, err0R2t1, wR2t1, new Dbl(0));
			Matrix point3DR2t2 = triangulate(R2t2, I, c, parallaxR2t2, err1R2t2, err0R2t2, wR2t2, new Dbl(0));

			// record parallax
			numHighParallax[0] += parallaxR1t1.getValue() > parallaxRequirement ? 1 : 0;
			numHighParallax[1] += parallaxR1t2.getValue() > parallaxRequirement ? 1 : 0;
			numHighParallax[2] += parallaxR2t1.getValue() > parallaxRequirement ? 1 : 0;
			numHighParallax[3] += parallaxR2t2.getValue() > parallaxRequirement ? 1 : 0;

			int numSelected = 0;
			if (point3DR1t1.get(2, 0) > 0 && wR1t1.getValue() > 0
					&& err0R1t1.getValue() < initializationReprojectionError
					&& err1R1t1.getValue() < initializationReprojectionError) {
				scores[0]++;
				numSelected++;
			}
			if (point3DR1t2.get(2, 0) > 0 && wR1t2.getValue() > 0
					&& err0R1t2.getValue() < initializationReprojectionError
					&& err1R1t2.getValue() < initializationReprojectionError) {
				scores[1]++;
				numSelected++;
			}
			if (point3DR2t1.get(2, 0) > 0 && wR2t1.getValue() > 0
					&& err0R2t1.getValue() < initializationReprojectionError
					&& err1R2t1.getValue() < initializationReprojectionError) {
				scores[2]++;
				numSelected++;
			}
			if (point3DR2t2.get(2, 0) > 0 && wR2t2.getValue() > 0
					&& err0R2t2.getValue() < initializationReprojectionError
					&& err1R2t2.getValue() < initializationReprojectionError) {
				scores[3]++;
				numSelected++;
			}
			if (numSelected > 1) {
				Utils.pl(
						"UH OH! More than one pose passed acceptance criteria in fundamental matrix initialization! (Photogrammetry::SFMFundamentalMatrixEstimate()) ==> numSelected: "
								+ numSelected);
			}

		}

		Utils.pl("scores: " + scores[0] + ", " + scores[1] + ", " + scores[2] + ", " + scores[3]);

		// find highest scoring pose
		int highestInd = 0;
		for (int i = 1; i < scores.length; i++) {
			highestInd = scores[i] > scores[highestInd] ? i : highestInd;
		}

		// set the final parallax amount
		numWithHighParallax.setValue(numHighParallax[highestInd]);

		goodPoints.setValue(scores[highestInd]);

		// convert to quaternion and pose object
		Matrix selection = possiblePoses[highestInd];
		DMatrixRMaj R = new DMatrixRMaj(3, 3);
		R.add(0, 0, selection.get(0, 0));
		R.add(0, 1, selection.get(0, 1));
		R.add(0, 2, selection.get(0, 2));
		R.add(1, 0, selection.get(1, 0));
		R.add(1, 1, selection.get(1, 1));
		R.add(1, 2, selection.get(1, 2));
		R.add(2, 0, selection.get(2, 0));
		R.add(2, 1, selection.get(2, 1));
		R.add(2, 2, selection.get(2, 2));

		Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(R, null);
		q.normalize();

		Pose pose = new Pose();
		pose.setQw(q.w);
		pose.setQx(q.x);
		pose.setQy(q.y);
		pose.setQz(q.z);

		pose.setT(selection.get(0, 3), selection.get(1, 3), selection.get(2, 3));

		return pose;

	}

	// return true if map point was able to be accurately retriangulated
	public static boolean triangulateMapPoint(MapPoint mapPoint, long frameNum, Dbl parallax, Dbl reprojErrorSecondary,
			Dbl reprojErrorPrimary, Dbl wSecondary, Dbl wPrimary) {

		double MAX_REPROJ_ERR = Parameters.<Double>get("initializationReprojectionError") * 3;

		if (mapPoint.getObservations().size() < 2) {
			return false;
		}

		// get widest baseline
		Observation bestObsv0 = null;
		Observation bestObsv1 = null;
		double bestBaseline = 0;
		for (int i = 0; i < mapPoint.getObservations().size() - 1; i++) {
			Observation o0 = mapPoint.getObservations().get(i);
			for (int j = 1; j < mapPoint.getObservations().size(); j++) {

				Observation o1 = mapPoint.getObservations().get(j);

				// band aid. somehow, there are observations that point to keyframes that have
				// no knowledge of them
				if (o0.getKeyframe().getMapPointIndices().get(mapPoint) == null
						|| o1.getKeyframe().getMapPointIndices().get(mapPoint) == null) {
					continue;
				}

				double dist = o0.getKeyframe().getPose().getDistanceFrom(o1.getKeyframe().getPose());

				if (dist > bestBaseline) {
					bestObsv0 = o0;
					bestObsv1 = o1;
					bestBaseline = dist;
				}
			}
		}

		// if bad baseline, stop
		if (bestBaseline < 1) {
//			Utils.pl("BAD BASELINE: " + bestBaseline);
			return false;
		}

		// prep input
		Point pt0 = bestObsv0.getKeyframe().getKeypointsList()
				.get(bestObsv0.getKeyframe().getMapPointIndices().get(mapPoint).get(0)).pt;
		Point pt1 = bestObsv1.getKeyframe().getKeypointsList()
				.get(bestObsv1.getKeyframe().getMapPointIndices().get(mapPoint).get(0)).pt;
		Correspondence2D2D c = new Correspondence2D2D();
		c.setX0(pt0.x);
		c.setY0(pt0.y);
		c.setX1(pt1.x);
		c.setY1(pt1.y);

		Matrix primaryPose = bestObsv0.getKeyframe().getPose().getHomogeneousMatrix();
		Matrix secondaryPose = bestObsv1.getKeyframe().getPose().getHomogeneousMatrix();

		Matrix newPoint = triangulate(secondaryPose, primaryPose, c, parallax, reprojErrorSecondary, reprojErrorPrimary,
				wSecondary, wPrimary);

		// if cheirality is violated or reprojection errors are high, stop
		if (wPrimary.getValue() < 0 || wSecondary.getValue() < 0 || reprojErrorPrimary.getValue() > MAX_REPROJ_ERR
				|| reprojErrorSecondary.getValue() > MAX_REPROJ_ERR) {
//			Utils.pl("wPrimary: " + wPrimary + "\t\twSecondary: " + wSecondary + "\t\treprojErrorPrimary: "
//					+ reprojErrorPrimary + "\t\treprojErrorSecondary: " + reprojErrorSecondary);
			return false;
		}

		// otherwise, update the point
		if (mapPoint.getPoint() == null) {
			mapPoint.setPoint(new Point3D(), frameNum);
		}
		mapPoint.getPoint().setX(newPoint.get(0, 0));
		mapPoint.getPoint().setY(newPoint.get(1, 0));
		mapPoint.getPoint().setZ(newPoint.get(2, 0));
		mapPoint.setFrameTriangulated(frameNum);

		return true;

	}

	public static Matrix triangulate(Matrix secondaryPose, Matrix primaryPose, Correspondence2D2D c, Dbl parallax,
			Dbl reprojErrorSecondary, Dbl reprojErrorPrimary, Dbl wSecondary, Dbl wPrimary) {

//		Matrix Pprime = E.times(pose);
		Matrix Pprime = Parameters.getK4x4().times(secondaryPose);

		Matrix P = Parameters.getK4x4().times(primaryPose);

		// compute A matrix for Ax = 0
		Matrix row0 = P.getMatrix(2, 2, 0, 3).times(c.getX0()).minus(P.getMatrix(0, 0, 0, 3));
		Matrix row1 = P.getMatrix(2, 2, 0, 3).times(c.getY0()).minus(P.getMatrix(1, 1, 0, 3));
		Matrix row2 = Pprime.getMatrix(2, 2, 0, 3).times(c.getX1()).minus(Pprime.getMatrix(0, 0, 0, 3));
		Matrix row3 = Pprime.getMatrix(2, 2, 0, 3).times(c.getY1()).minus(Pprime.getMatrix(1, 1, 0, 3));

		Matrix A = new Matrix(4, 4);
		A.setMatrix(0, 0, 0, 3, row0);
		A.setMatrix(1, 1, 0, 3, row1);
		A.setMatrix(2, 2, 0, 3, row2);
		A.setMatrix(3, 3, 0, 3, row3);

		SingularValueDecomposition svd = A.svd();
		Matrix X = svd.getV().getMatrix(0, 3, 3, 3);
		X = X.times(1.0 / X.get(3, 0));
		// System.out.println("X");
		// X.print(5, 4);

		// set parallax
		Matrix u = X.getMatrix(0, 2, 0, 0).minus(
				primaryPose.getMatrix(0, 2, 0, 2).transpose().times(primaryPose.getMatrix(0, 2, 3, 3).times(-1)));
		Matrix v = X.getMatrix(0, 2, 0, 0).minus(
				secondaryPose.getMatrix(0, 2, 0, 2).transpose().times(secondaryPose.getMatrix(0, 2, 3, 3).times(-1)));
		double cosParallax = u.transpose().times(v).get(0, 0) / (u.normF() * v.normF());

		double parallaxDegrees = Math.acos(cosParallax) * 180 / Math.PI;

		parallax.setValue(parallaxDegrees);

		// get reprojection errors
		Matrix p0 = new Matrix(3, 1);
		p0.set(0, 0, c.getX0());
		p0.set(1, 0, c.getY0());
		p0.set(2, 0, 1);
		Matrix p1 = new Matrix(3, 1);
		p1.set(0, 0, c.getX1());
		p1.set(1, 0, c.getY1());
		p1.set(2, 0, 1);

		Matrix proj0 = P.times(X);
		wPrimary.setValue(proj0.get(2, 0));
		proj0 = proj0.times(1 / proj0.get(2, 0));
		double reproj0 = p0.minus(proj0.getMatrix(0, 2, 0, 0)).normF();

		Matrix proj1 = Pprime.times(X);
		wSecondary.setValue(proj1.get(2, 0));
		proj1 = proj1.times(1 / proj1.get(2, 0));
		double reproj1 = p1.minus(proj1.getMatrix(0, 2, 0, 0)).normF();

		reprojErrorPrimary.setValue(reproj0);
		reprojErrorSecondary.setValue(reproj1);

		return X;
	}

	public static Matrix OpenCVPnP(List<Point3> point3s, List<Point> points, Mat rvec, Mat tvec,
			List<Integer> inlierIndices, boolean optimize) {
		return OpenCVPnP(point3s, points, rvec, tvec, inlierIndices, optimize, true);
	}

	public static Matrix OpenCVPnP(List<Point3> point3s, List<Point> points, Mat rvec, Mat tvec,
			List<Integer> inlierIndices, boolean optimize, boolean useExtrinsicGuess) {

		float ransacReprojThreshold = Parameters.<Float>get("PnPRansacReprojectionThreshold");

		if (inlierIndices == null) {
			inlierIndices = new ArrayList<Integer>();
		}

		inlierIndices.clear();

		MatOfPoint3f objectPoints = new MatOfPoint3f();
		objectPoints.fromList(point3s);
		MatOfPoint2f imagePoints = new MatOfPoint2f();
		imagePoints.fromList(points);
		Mat cameraMatrix = Parameters.getKMat();

		Mat inliers = new Mat();
		Calib3d.solvePnPRansac(objectPoints, imagePoints, cameraMatrix, new MatOfDouble(), rvec, tvec,
				useExtrinsicGuess, 100, ransacReprojThreshold, 0.99, inliers, Calib3d.SOLVEPNP_ITERATIVE);

		Utils.pl("Initial num of object points: " + objectPoints.rows());
		Utils.pl("Num inliers: " + inliers.rows());

		// if theres enough inliers, refine the estimate
		if (inliers.rows() > 10 && true) {
			// load inliers
			List<Point3> objectPointInliers = new ArrayList<Point3>();
			List<Point> imagePointInliers = new ArrayList<Point>();
			int[] inlierBuffer = new int[inliers.rows()];
			inliers.get(0, 0, inlierBuffer);
			for (int i = 0; i < inlierBuffer.length; i++) {
				inlierIndices.add(inlierBuffer[i]);
				objectPointInliers.add(point3s.get(inlierBuffer[i]));
				imagePointInliers.add(points.get(inlierBuffer[i]));
			}

			objectPoints.fromList(objectPointInliers);
			imagePoints.fromList(imagePointInliers);

			if (optimize) {
				Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, new MatOfDouble(), rvec, tvec, true,
						Calib3d.SOLVEPNP_ITERATIVE);
			}

		}

		Mat RMat = new Mat();
		Calib3d.Rodrigues(rvec, RMat);
		double[] RBuffer = new double[9];
		RMat.get(0, 0, RBuffer);
		double[] tBuffer = new double[3];
		tvec.get(0, 0, tBuffer);

		Matrix E = Matrix.identity(4, 4);
		E.set(0, 0, RBuffer[0]);
		E.set(0, 1, RBuffer[1]);
		E.set(0, 2, RBuffer[2]);
		E.set(1, 0, RBuffer[3]);
		E.set(1, 1, RBuffer[4]);
		E.set(1, 2, RBuffer[5]);
		E.set(2, 0, RBuffer[6]);
		E.set(2, 1, RBuffer[7]);
		E.set(2, 2, RBuffer[8]);
		E.set(0, 3, tBuffer[0]);
		E.set(1, 3, tBuffer[1]);
		E.set(2, 3, tBuffer[2]);

		return E;
	}

	public static void epipolarPrune(List<Correspondence2D2D> correspondences, Pose pose1, Pose pose0) {
		List<MapPoint> dummyMapPoints = new ArrayList<MapPoint>(correspondences.size());
		for (int i = 0; i < correspondences.size(); i++) {
			dummyMapPoints.add(null);
		}
		epipolarPrune(correspondences, dummyMapPoints, pose1, pose0);
	}

	// using epipolar search, return a list of correspondences that satisfies
	// epipolar constraint
	public static void epipolarPrune(List<Correspondence2D2D> correspondences, List<MapPoint> correspondenceMapPoints,
			Pose pose1, Pose pose0) {

		double THRESHOLD = 1;

		// get the second pose with respect to the first pose
		Matrix E = pose1.getHomogeneousMatrix().times(pose0.getHomogeneousMatrix().inverse());

		// get skew-symmetric cross product representation of the baseline
		double a1 = E.get(0, 3);
		double a2 = E.get(1, 3);
		double a3 = E.get(2, 3);
		Matrix B = new Matrix(3, 3);
		B.set(0, 1, -a3);
		B.set(0, 2, a2);
		B.set(1, 0, a3);
		B.set(1, 2, -a1);
		B.set(2, 0, -a2);
		B.set(2, 1, a1);

		// get essential matrix
		Matrix essentialMatrix = B.times(E.getMatrix(0, 2, 0, 2));

		// get fundamental matrix
		Matrix KInv = Parameters.getK().inverse();
		Matrix fundamentalMatrix = KInv.transpose().times(essentialMatrix).times(KInv);

		double max = 0;

		List<Double> epi = new ArrayList<Double>();

		// evaluate each point with epipolar constraint
		int numDiscarded = 0;
		for (int i = 0; i < correspondences.size(); i++) {
			Correspondence2D2D c = correspondences.get(i);
			Matrix point0 = new Matrix(3, 1);
			point0.set(0, 0, c.getX0());
			point0.set(1, 0, c.getY0());
			point0.set(2, 0, 1);
			Matrix point1 = new Matrix(3, 1);
			point1.set(0, 0, c.getX1());
			point1.set(1, 0, c.getY1());
			point1.set(2, 0, 1);

			double epipolar = Math.abs(point1.transpose().times(fundamentalMatrix).times(point0).get(0, 0));
			epi.add(epipolar);
//			Utils.pl("epipolar: " + epipolar);
			max = epipolar > max ? epipolar : max;
			if (epipolar > THRESHOLD) {
				correspondences.remove(i);
				correspondenceMapPoints.remove(i);
				numDiscarded++;
				i--;
			}
		}

//		Utils.pl("epipolar prune, num discarded: " + numDiscarded);
//		Utils.pl("Max epipolar: " + max);

		Collections.sort(epi);
		for (int i = 0; i < epi.size(); i++) {
//			Utils.pl("epipolar: " + epi.get(i));
		}

	}

	public static double getReprojectionError(Pose pose, Point3D p3, Point p2, Matrix K4x4) {
//		Utils.pl("pose: " + pose);
//		Utils.pl("p3: " + p3);
//		Utils.pl("p2: " + p2);
//		Utils.pl("K4x4: " + K4x4);
		Matrix proj = K4x4.times(pose.getHomogeneousMatrix()).times(p3.getHomogeneousMatrix());
		proj = proj.times(1 / proj.get(2, 0));
		return Math.sqrt(Math.pow(proj.get(0, 0) - p2.x, 2) + Math.pow(proj.get(1, 0) - p2.y, 2));
	}

	public static void triangulateUntrackedMapPoints(long frameNum, Pose currentPose,
			List<Correspondence2D2D> untriangulatedCorrespondences, List<MapPoint> untriangulatedMapPoints, Map map) {

//		Utils.pl("*******************************************************************");
//		Utils.pl("**********************  TRIANGULATING...   ************************");
//		Utils.pl("*******************************************************************");

		float reprojError = 4f;
		double BASELINE_REQUIREMENT = Parameters.<Double>get("triangulationBaseline");

		// get point triangulations
		List<Point3D> newPoints = new ArrayList<Point3D>();
		List<Keyframe> baseKeyframes = new ArrayList<Keyframe>();
		List<Correspondence2D2D> updatedCorrespondences = new ArrayList<Correspondence2D2D>();
		List<MapPoint> updatedMapPoints = new ArrayList<MapPoint>();
		for (int i = 0; i < untriangulatedCorrespondences.size(); i++) {
			Correspondence2D2D c = untriangulatedCorrespondences.get(i);

			// check if mapPoint has any observations (if the map point has undergone
			// pruning, it might not)
			if (untriangulatedMapPoints.get(i).getObservations().size() == 0) {
				continue;
			}

			// find oldest keyframe
			Keyframe oldestKeyframe = untriangulatedMapPoints.get(i).getObservations().get(0).getKeyframe();

			// check distance between keyframe and currentPose. if too small, omit.
			if (currentPose.getDistanceFrom(oldestKeyframe.getPose()) < BASELINE_REQUIREMENT) {
				continue;
			}

			// create new correspondence that uses observation from old keyframe
			Correspondence2D2D wideC = new Correspondence2D2D();
			wideC.setX0(untriangulatedMapPoints.get(i).getObservations().get(0).getPoint().x);
			wideC.setY0(untriangulatedMapPoints.get(i).getObservations().get(0).getPoint().y);
			wideC.setX1(c.getX1());
			wideC.setY1(c.getY1());

			Dbl parallax = new Dbl(0);
			Dbl err0 = new Dbl(0);
			Dbl err1 = new Dbl(0);
			Dbl w0 = new Dbl(0);
			Dbl w1 = new Dbl(0);
			Matrix pointMatrix = Photogrammetry.triangulate(currentPose.getHomogeneousMatrix(),
					oldestKeyframe.getPose().getHomogeneousMatrix(), wideC, parallax, err1, err0, w1, w0);

			// throw out if parallax is less than 1 degree
			if (parallax.getValue() < 1) {
				continue;
			}

			// omit if point is behind cameras
			if (w0.getValue() < 0 || w1.getValue() < 0) {
				continue;
			}

			// omit if reprojection error is too high
			if (err0.getValue() > reprojError || err1.getValue() > reprojError) {
				continue;
			}

			// register this point
			Point3D point3D = new Point3D(pointMatrix.get(0, 0), pointMatrix.get(1, 0), pointMatrix.get(2, 0));
			newPoints.add(point3D);
			baseKeyframes.add(oldestKeyframe);
			updatedMapPoints.add(untriangulatedMapPoints.get(i));
			updatedCorrespondences.add(wideC);
		}

		// set each point in the map
		for (int i = 0; i < newPoints.size(); i++) {

			// set the point
			Point3D point3D = newPoints.get(i);
			synchronized (map) {
				updatedMapPoints.get(i).setPoint(point3D, frameNum);
				map.registerPoint(point3D);
			}
		}

		// cluster map points
		map.partitionMapPoints(updatedMapPoints);

		// add map points to pruning list
		synchronized (map) {
			map.getUncheckedMapPoints().addAll(updatedMapPoints);
		}

	}

	public static Pose SFMHomographyEstimate(List<Correspondence2D2D> correspondences, List<Boolean> inlierMap,
			Dbl numWithHighParallax) {

		Mat homography = estimateHomography(correspondences, inlierMap);
		Pose initialPose = new Pose();
		Pose pose = getPoseFromHomography(homography, initialPose, correspondences, inlierMap, numWithHighParallax);
		return pose;

	}

	public static Mat estimateHomography(List<Correspondence2D2D> correspondences, List<Boolean> inlierMap) {

		double ransacReprojThreshold = 0.25;

		ArrayList<Point> matchedKeyframePoints = new ArrayList<Point>();
		ArrayList<Point> matchedPoints = new ArrayList<Point>();

		for (int i = 0; i < correspondences.size(); i++) {
			Correspondence2D2D c = correspondences.get(i);
			Point point1 = new Point();
			Point point2 = new Point();
			point1.x = c.getX0();
			point1.y = c.getY0();
			point2.x = c.getX1();
			point2.y = c.getY1();
			matchedKeyframePoints.add(point1);
			matchedPoints.add(point2);
		}

		// compute homography
		MatOfPoint2f keyframeMat = new MatOfPoint2f();
		MatOfPoint2f matKeypoints = new MatOfPoint2f();
		keyframeMat.fromList(matchedKeyframePoints);
		matKeypoints.fromList(matchedPoints);
		Mat homography = Calib3d.findHomography(keyframeMat, matKeypoints, Calib3d.RANSAC, ransacReprojThreshold, null,
				500, 0.99);

		// build inlierMap
		Matrix homMatrix = Utils.MatToMatrix(homography);
		inlierMap.clear();

		for (int i = 0; i < correspondences.size(); i++) {
			Correspondence2D2D c = correspondences.get(i);
			Matrix x0 = new Matrix(3, 1);
			x0.set(0, 0, c.getX0());
			x0.set(1, 0, c.getY0());
			x0.set(2, 0, 1);
			Matrix x1 = new Matrix(3, 1);
			x1.set(0, 0, c.getX1());
			x1.set(1, 0, c.getY1());
			x1.set(2, 0, 1);

			Matrix projectedX1 = homMatrix.times(x0);
			projectedX1 = projectedX1.times(1 / projectedX1.get(2, 0));

			double error = x1.minus(projectedX1).normF();
			inlierMap.add(error < ransacReprojThreshold);

		}

		return homography;

	}

	public static Pose getPoseFromHomography(Mat homography, Pose primaryCamera,
			List<Correspondence2D2D> correspondences, List<Boolean> inlierMap, Dbl numWithHighParallax) {

		Mat intrinsics = Parameters.getKMat();
		List<Mat> rotations = new ArrayList<Mat>();
		List<Mat> translations = new ArrayList<Mat>();
		List<Mat> normals = new ArrayList<Mat>();
		Calib3d.decomposeHomographyMat(homography, intrinsics, rotations, translations, normals);

		Matrix E = selectHomographySolution(primaryCamera, rotations, translations, correspondences, inlierMap,
				numWithHighParallax);

		return Utils.EtoPose(E);
	}

	public static Matrix selectHomographySolution(Pose primaryCamera, List<Mat> rotations, List<Mat> translations,
			List<Correspondence2D2D> correspondences, List<Boolean> inlierMap, Dbl numWithHighParallax) {

		double parallaxRequirement = Parameters.<Double>get("parallaxRequirement");

		if (rotations.size() == 1) {
			Matrix R = Utils.MatToMatrix(rotations.get(0));
			Matrix t = Utils.MatToMatrix(translations.get(0));
			Matrix E = Matrix.identity(4, 4);
			E.set(0, 0, R.get(0, 0));
			E.set(0, 1, R.get(0, 1));
			E.set(0, 2, R.get(0, 2));
			E.set(1, 0, R.get(1, 0));
			E.set(1, 1, R.get(1, 1));
			E.set(1, 2, R.get(1, 2));
			E.set(2, 0, R.get(2, 0));
			E.set(2, 1, R.get(2, 1));
			E.set(2, 2, R.get(2, 2));
			E.set(0, 3, t.get(0, 0));
			E.set(1, 3, t.get(1, 0));
			E.set(2, 3, t.get(2, 0));
			return E;
		}
		Matrix pose = primaryCamera.getHomogeneousMatrix();
		Matrix R1 = Utils.MatToMatrix(rotations.get(0));
		Matrix R2 = Utils.MatToMatrix(rotations.get(1));
		Matrix R3 = Utils.MatToMatrix(rotations.get(2));
		Matrix R4 = Utils.MatToMatrix(rotations.get(3));

		Matrix t1 = Utils.MatToMatrix(translations.get(0));
		Matrix t2 = Utils.MatToMatrix(translations.get(1));
		Matrix t3 = Utils.MatToMatrix(translations.get(2));
		Matrix t4 = Utils.MatToMatrix(translations.get(3));

		// set up extrinsic matrices (all possible options)
		Matrix E1 = Matrix.identity(4, 4);
		Matrix E2 = Matrix.identity(4, 4);
		Matrix E3 = Matrix.identity(4, 4);
		Matrix E4 = Matrix.identity(4, 4);

		E1.set(0, 0, R1.get(0, 0));
		E1.set(0, 1, R1.get(0, 1));
		E1.set(0, 2, R1.get(0, 2));
		E1.set(1, 0, R1.get(1, 0));
		E1.set(1, 1, R1.get(1, 1));
		E1.set(1, 2, R1.get(1, 2));
		E1.set(2, 0, R1.get(2, 0));
		E1.set(2, 1, R1.get(2, 1));
		E1.set(2, 2, R1.get(2, 2));
		E1.set(0, 3, t1.get(0, 0));
		E1.set(1, 3, t1.get(1, 0));
		E1.set(2, 3, t1.get(2, 0));

		E2.set(0, 0, R2.get(0, 0));
		E2.set(0, 1, R2.get(0, 1));
		E2.set(0, 2, R2.get(0, 2));
		E2.set(1, 0, R2.get(1, 0));
		E2.set(1, 1, R2.get(1, 1));
		E2.set(1, 2, R2.get(1, 2));
		E2.set(2, 0, R2.get(2, 0));
		E2.set(2, 1, R2.get(2, 1));
		E2.set(2, 2, R2.get(2, 2));
		E2.set(0, 3, t2.get(0, 0));
		E2.set(1, 3, t2.get(1, 0));
		E2.set(2, 3, t2.get(2, 0));

		E3.set(0, 0, R3.get(0, 0));
		E3.set(0, 1, R3.get(0, 1));
		E3.set(0, 2, R3.get(0, 2));
		E3.set(1, 0, R3.get(1, 0));
		E3.set(1, 1, R3.get(1, 1));
		E3.set(1, 2, R3.get(1, 2));
		E3.set(2, 0, R3.get(2, 0));
		E3.set(2, 1, R3.get(2, 1));
		E3.set(2, 2, R3.get(2, 2));
		E3.set(0, 3, t3.get(0, 0));
		E3.set(1, 3, t3.get(1, 0));
		E3.set(2, 3, t3.get(2, 0));

		E4.set(0, 0, R4.get(0, 0));
		E4.set(0, 1, R4.get(0, 1));
		E4.set(0, 2, R4.get(0, 2));
		E4.set(1, 0, R4.get(1, 0));
		E4.set(1, 1, R4.get(1, 1));
		E4.set(1, 2, R4.get(1, 2));
		E4.set(2, 0, R4.get(2, 0));
		E4.set(2, 1, R4.get(2, 1));
		E4.set(2, 2, R4.get(2, 2));
		E4.set(0, 3, t4.get(0, 0));
		E4.set(1, 3, t4.get(1, 0));
		E4.set(2, 3, t4.get(2, 0));

		List<Matrix> poses = new ArrayList<Matrix>();
		poses.add(E1);
		poses.add(E2);
		poses.add(E3);
		poses.add(E4);

		int[] scores = { 0, 0, 0, 0 };
		double[] reprojErrors = { 0, 0, 0, 0 };

		// additionally, record the number of points that triangulated with sufficient
		// parallax for each hypothesis
		int[] numHighParallax = { 0, 0, 0, 0 };

		for (int i = 0; i < correspondences.size(); i++) {

			Correspondence2D2D c = correspondences.get(i);

			if (!inlierMap.get(i)) {
				continue;
			}

			// record parallax values
			Dbl parallaxX1 = new Dbl(0);
			Dbl parallaxX2 = new Dbl(0);
			Dbl parallaxX3 = new Dbl(0);
			Dbl parallaxX4 = new Dbl(0);

			// reprojection errors
			Dbl err0X1 = new Dbl(0);
			Dbl err1X1 = new Dbl(0);
			Dbl err0X2 = new Dbl(0);
			Dbl err1X2 = new Dbl(0);
			Dbl err0X3 = new Dbl(0);
			Dbl err1X3 = new Dbl(0);
			Dbl err0X4 = new Dbl(0);
			Dbl err1X4 = new Dbl(0);

			// w values of the reprojections (for cheirality check)
			Dbl w0X1 = new Dbl(0);
			Dbl w1X1 = new Dbl(0);
			Dbl w0X2 = new Dbl(0);
			Dbl w1X2 = new Dbl(0);
			Dbl w0X3 = new Dbl(0);
			Dbl w1X3 = new Dbl(0);
			Dbl w0X4 = new Dbl(0);
			Dbl w1X4 = new Dbl(0);

			// triangulated points
			Matrix X1 = triangulate(E1.times(pose), pose, c, parallaxX1, err1X1, err0X1, w1X1, w0X1);
			Matrix X2 = triangulate(E2.times(pose), pose, c, parallaxX2, err1X2, err0X2, w1X2, w0X2);
			Matrix X3 = triangulate(E3.times(pose), pose, c, parallaxX3, err1X3, err0X3, w1X3, w0X3);
			Matrix X4 = triangulate(E4.times(pose), pose, c, parallaxX4, err1X4, err0X4, w1X4, w0X4);

			// record parallax results
			numHighParallax[0] += parallaxX1.getValue() > parallaxRequirement ? 1 : 0;
			numHighParallax[1] += parallaxX2.getValue() > parallaxRequirement ? 1 : 0;
			numHighParallax[2] += parallaxX3.getValue() > parallaxRequirement ? 1 : 0;
			numHighParallax[3] += parallaxX4.getValue() > parallaxRequirement ? 1 : 0;

			reprojErrors[0] += err1X1.getValue();
			reprojErrors[1] += err1X2.getValue();
			reprojErrors[2] += err1X3.getValue();
			reprojErrors[3] += err1X4.getValue();

			if (w0X1.getValue() > 0 && w1X1.getValue() > 0) {
				scores[0]++;
			}

			if (w0X2.getValue() > 0 && w1X2.getValue() > 0) {
				scores[1]++;
			}

			if (w0X3.getValue() > 0 && w1X3.getValue() > 0) {
				scores[2]++;
			}

			if (w0X4.getValue() > 0 && w1X4.getValue() > 0) {
				scores[3]++;
			}
		}

		// narrow down options based on cheirality (should have 2 hypotheses
		// remaining)
		ArrayList<Integer> bestHypothesesInd = new ArrayList<Integer>();
		double avgScore = (scores[0] + scores[1] + scores[2] + scores[3]) / 4.0;

		for (int i = 0; i < scores.length; i++) {
			int score = scores[i];
			if (score > avgScore) {
				bestHypothesesInd.add(i);
			}
		}

		// pick hypothesis based on reprojection error
		int lowestReprojInd = bestHypothesesInd.get(0);
		for (int i = 1; i < bestHypothesesInd.size(); i++) {
			if (reprojErrors[bestHypothesesInd.get(i)] < reprojErrors[lowestReprojInd]) {
				lowestReprojInd = bestHypothesesInd.get(i);
			}
		}

		// record high parallax count
		numWithHighParallax.setValue(numHighParallax[lowestReprojInd]);

		// finally, set the correct decomposition
		Matrix solution = E1;
		if (lowestReprojInd == 0) {
			solution = E1;
		} else if (lowestReprojInd == 1) {
			solution = E2;
		} else if (lowestReprojInd == 2) {
			solution = E3;
		} else if (lowestReprojInd == 3) {
			solution = E4;
		}

		return solution;

	}

}
