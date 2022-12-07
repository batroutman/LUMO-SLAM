package toolbox;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.joml.Matrix4f;
import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;

import Jama.Matrix;
import Jama.SingularValueDecomposition;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.so.Quaternion_F64;
import lumoslam.MapPoint;
import types.Dbl;
import types.Point3D;
import types.Pose;

public class Utils {

	public static void pl(Object obj) {
		System.out.println(obj);
	}

	public static void p(Object obj) {
		System.out.print(obj);
	}

	public static void pa(byte[] array) {
		System.out.println(java.util.Arrays.toString(array));
	}

	public static <T> void pa(List<T> list) {
		System.out.println(Arrays.toString(list.toArray()));
	}

	public static double pointDistance(Point3D p0, Point3D p1) {
		return Math.sqrt(Math.pow(p0.getX() - p1.getX(), 2) + Math.pow(p0.getY() - p1.getY(), 2)
				+ Math.pow(p0.getZ() - p1.getZ(), 2));
	}

	// return descriptors in list form (still in byte representation)
	public static List<List<Byte>> getDescriptorByteLists(Mat descriptors) {
		byte[] buffer = new byte[descriptors.rows() * 32];
		descriptors.get(0, 0, buffer);
		List<List<Byte>> byteLists = new ArrayList<List<Byte>>();
		for (int row = 0; row < descriptors.rows(); row++) {
			List<Byte> byteList = new ArrayList<Byte>();
			for (int col = 0; col < 32; col++) {
				byteList.add(buffer[row * 32 + col]);
			}
			byteLists.add(byteList);
		}
		return byteLists;
	}

	public static Mat byteListsToMat(List<List<Byte>> byteLists) {
		if (byteLists.size() == 0) {
			return new Mat(0, 0, CvType.CV_8U);
		}
		int rows = byteLists.size();
		int cols = byteLists.get(0).size();
		Mat mat = new Mat(rows, cols, CvType.CV_8U);
		byte[] buffer = new byte[rows * cols];
		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < cols; j++) {
				buffer[i * cols + j] = byteLists.get(i).get(j);
			}
		}
		mat.put(0, 0, buffer);
		return mat;
	}

	// given a pose and an (x,y,z) vector of a point, return a rotation (x,y,z)
	// that would point a model at the point towards the camera
	public static float[] getRotationDeg(Pose pose, Point3D point) {

		float[] rotations = new float[3];

		// euclidean distance
		double dist = Math.sqrt(Math.pow(point.getX() - pose.getCx(), 2) + Math.pow(point.getY() - pose.getCy(), 2)
				+ Math.pow(point.getZ() - pose.getCz(), 2));

		if (dist == 0) {
			rotations[0] = 0;
			rotations[1] = 0;
			rotations[2] = 0;
			return rotations;
		}

		double dx = point.getX() - pose.getCx();
		double dy = point.getY() - pose.getCy();
		double dz = point.getZ() - pose.getCz();

		rotations[0] = (float) (Math.asin(dy / dist) * 180 / Math.PI);
		rotations[1] = (float) (Math.asin(dx / dist) * 180 / Math.PI);
		rotations[2] = 0;// (float) pose.getRotZDeg();

		// if dz is negative, reflect rotations
		if (dz < 0) {

//			rotations[0] = 180 - rotations[0];
			rotations[1] = 180 - rotations[1];
		}

		return rotations;

	}

	// return cross product (column vectors) of a and b (also column vectors)
	public static Matrix crossProduct(Matrix a, Matrix b) {
		Matrix cross = new Matrix(3, 1);
		double a1 = a.get(0, 0);
		double a2 = a.get(1, 0);
		double a3 = a.get(2, 0);
		double b1 = b.get(0, 0);
		double b2 = b.get(1, 0);
		double b3 = b.get(2, 0);

		cross.set(0, 0, a2 * b3 - a3 * b2);
		cross.set(1, 0, a3 * b1 - a1 * b3);
		cross.set(2, 0, a1 * b2 - a2 * b1);
		return cross;
	}

	public static byte[] byteArray(List<Byte> bytesList) {
		byte[] bytes = new byte[bytesList.size()];
		for (int i = 0; i < bytesList.size(); i++) {
			bytes[i] = bytesList.get(i);
		}
		return bytes;
	}

	public static Matrix pseudoInverse(Matrix m) {
		Matrix matrix = m.copy();
		if (m.getRowDimension() < m.getColumnDimension()) {
			matrix = matrix.transpose();
		}
		SingularValueDecomposition svd = matrix.svd();
		Matrix invertedSigma = svd.getS().copy();
		int dim = invertedSigma.getRowDimension();
		for (int i = 0; i < dim; i++) {
			if (invertedSigma.get(i, i) != 0) {
				invertedSigma.set(i, i, 1 / invertedSigma.get(i, i));
			}
		}
		Matrix pseudoInverse = svd.getV().times(invertedSigma).times(svd.getU().transpose());
		if (m.getRowDimension() < m.getColumnDimension()) {
			pseudoInverse = pseudoInverse.transpose();
		}
		return pseudoInverse;
	}

	public static void poseToRodrigues(Pose pose, Mat rvec, Mat tvec) {
		Matrix homMatrix = pose.getHomogeneousMatrix();
		Matrix R = homMatrix.getMatrix(0, 2, 0, 2);
		Matrix left = R.minus(R.transpose()).times(0.5);
		Matrix rMatrix = new Matrix(3, 1);
		rMatrix.set(0, 0, left.get(2, 1));
		rMatrix.set(1, 0, left.get(0, 2));
		rMatrix.set(2, 0, left.get(1, 0));
		double sinTheta = rMatrix.normF();
		double theta = Math.asin(sinTheta);
		rMatrix = rMatrix.times(theta);

		rvec.create(3, 1, CvType.CV_64F);
		tvec.create(3, 1, CvType.CV_64F);
		rvec.put(0, 0, rMatrix.get(0, 0), rMatrix.get(1, 0), rMatrix.get(2, 0));
		tvec.put(0, 0, homMatrix.get(0, 3), homMatrix.get(1, 3), homMatrix.get(2, 3));
	}

	// provide a text-based histogram of the numeric data
	public static String textHistogram(List<Double> data, double binSize, int maxHeight) {

		// get min and max values
		double minValue = Double.MAX_VALUE;
		double maxValue = Double.MIN_VALUE;
		for (Double d : data) {
			if (d < minValue) {
				minValue = d;
			}
			if (d > maxValue) {
				maxValue = d;
			}
		}

		// initialize bins
		List<Integer> bins = new ArrayList<Integer>();
		for (int i = 0; i < (maxValue - minValue) / binSize + 1; i++) {
			bins.add(0);
		}

		// populate bins
		for (Double d : data) {
			int binNum = (int) ((d - minValue) / binSize);
			bins.set(binNum, bins.get(binNum) + 1);
		}

		// get max capacity
		int maxCapacity = 0;
		for (Integer num : bins) {
			if (num > maxCapacity) {
				maxCapacity = num;
			}
		}

		// construct output string
		String histogram = "";
		for (int i = 0; i < bins.size(); i++) {
			double labelNum = minValue + i * binSize;
			histogram += String.format("%.4f : ", labelNum);
			int numAsterisks = (int) ((double) bins.get(i) / maxCapacity * maxHeight);
			for (int j = 0; j < numAsterisks; j++) {
				histogram += "*";
			}
			histogram += "\n";
		}

		return histogram;

	}

	// given 2 matrices of binary descriptors (single row, multiple columns),
	// possibly encoded in 8-bit sequences, return the hamming distance of the
	// matrices
	public static int descriptorDistance(Matrix descriptor1, Matrix descriptor2) {
		int total = 0;
		for (int i = 0; i < descriptor1.getColumnDimension(); i++) {
			total += Integer.bitCount((int) descriptor1.get(0, i) ^ (int) descriptor2.get(0, i));
		}
		return total;
	}

	// euclidean distance between 2 poses' spatial locations in the same space
	public static double poseDistance(Pose p0, Pose p1) {
		double dx = p1.getCx() - p0.getCx();
		double dy = p1.getCy() - p0.getCy();
		double dz = p1.getCz() - p0.getCz();
		return Math.sqrt(dx * dx + dy * dy + dz * dz);
	}

	// using constant velocity motion model (where poseVelocity is the
	// transformation per frame), provide an estimate of the pose
	public static Pose getPoseEstimate(long initialFrame, Pose initialPose, Pose poseVelocity, long currentFrame) {
		Pose estimatedPose = initialPose;
		int frames = (int) (currentFrame - initialFrame);
		for (int i = 0; i < frames; i++) {
			estimatedPose = applyPoseTransform(initialPose, poseVelocity);
		}
		return estimatedPose;
	}

	// return the pose p2 such that p2 = p1 * p0
	public static Pose applyPoseTransform(Pose pose0, Pose pose1) {
		// handle quaternions
		Matrix q0 = new Matrix(4, 1);
		q0.set(0, 0, pose0.getQw());
		q0.set(1, 0, pose0.getQx());
		q0.set(2, 0, pose0.getQy());
		q0.set(3, 0, pose0.getQz());

		Matrix q1 = new Matrix(4, 1);
		q1.set(0, 0, pose1.getQw());
		q1.set(1, 0, pose1.getQx());
		q1.set(2, 0, pose1.getQy());
		q1.set(3, 0, pose1.getQz());

		Matrix q2 = quatMult(q1, q0);

		// calculate translation
		double cx = pose0.getCx() + pose1.getCx();
		double cy = pose0.getCy() + pose1.getCy();
		double cz = pose0.getCz() + pose1.getCz();

		// load pose
		Pose p2 = new Pose();
		p2.setQw(q2.get(0, 0));
		p2.setQx(q2.get(1, 0));
		p2.setQy(q2.get(2, 0));
		p2.setQz(q2.get(3, 0));
		p2.setCx(cx);
		p2.setCy(cy);
		p2.setCz(cz);
		return p2;
	}

	// return a Pose P such that P * pose0 = pose1
	public static Pose getPoseDifference(Pose pose0, Pose pose1) {
		Matrix q1 = new Matrix(4, 1);
		q1.set(0, 0, pose1.getQw());
		q1.set(1, 0, pose1.getQx());
		q1.set(2, 0, pose1.getQy());
		q1.set(3, 0, pose1.getQz());

		// invert the initial pose quaternion
		Matrix q0Inv = new Matrix(4, 1);
		q0Inv.set(0, 0, -pose0.getQw());
		q0Inv.set(1, 0, pose0.getQx());
		q0Inv.set(2, 0, pose0.getQy());
		q0Inv.set(3, 0, pose0.getQz());

		// calculate the new rotation difference from pose0 -> pose1
		Matrix newQuat = quatMult(q1, q0Inv);

		// calculate absolute translation difference
		double cx = pose1.getCx() - pose0.getCx();
		double cy = pose1.getCy() - pose0.getCy();
		double cz = pose1.getCz() - pose0.getCz();

		Pose newPose = new Pose();
		newPose.setQw(newQuat.get(0, 0));
		newPose.setQx(newQuat.get(1, 0));
		newPose.setQy(newQuat.get(2, 0));
		newPose.setQz(newQuat.get(3, 0));
		newPose.setCx(cx);
		newPose.setCy(cy);
		newPose.setCz(cz);
		return newPose;
	}

	public static void getTrackedKeypoints(List<KeyPoint> keypoints, List<MapPoint> mapPointPerDescriptor,
			List<KeyPoint> outTrackedKeypoints) {

		outTrackedKeypoints.clear();
		for (int i = 0; i < keypoints.size(); i++) {
			if (mapPointPerDescriptor.get(i) != null) {
				outTrackedKeypoints.add(keypoints.get(i));
			}
		}

	}

	// return a list without null values
	public static <T> List<T> compressList(List<T> list) {
		List<T> newList = new ArrayList<T>();
		for (T item : list) {
			if (item != null) {
				newList.add(item);
			}
		}
		return newList;
	}

	// convert extrinsic matrix E to Pose type
	public static Pose EtoPose(Matrix E) {
		DMatrixRMaj R = new DMatrixRMaj(3, 3);
		R.add(0, 0, E.get(0, 0));
		R.add(0, 1, E.get(0, 1));
		R.add(0, 2, E.get(0, 2));
		R.add(1, 0, E.get(1, 0));
		R.add(1, 1, E.get(1, 1));
		R.add(1, 2, E.get(1, 2));
		R.add(2, 0, E.get(2, 0));
		R.add(2, 1, E.get(2, 1));
		R.add(2, 2, E.get(2, 2));

		Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(R, null);
		q.normalize();

		Pose pose = new Pose();
		pose.setQw(q.w);
		pose.setQx(q.x);
		pose.setQy(q.y);
		pose.setQz(q.z);

		pose.setT(E.get(0, 3), E.get(1, 3), E.get(2, 3));
		return pose;
	}

	public static Mat mergeDescriptorList(List<Mat> descriptorList) {
		byte[] buffer = new byte[descriptorList.size() * 32];

		for (int i = 0; i < descriptorList.size(); i++) {
			byte[] bufferTemp = new byte[32];
			descriptorList.get(i).get(0, 0, bufferTemp);
			for (int j = 0; j < 32; j++) {
				buffer[i * 32 + j] = bufferTemp[j];
			}
		}

		Mat descriptors = new Mat(descriptorList.size(), 32, CvType.CV_8U);
		descriptors.put(0, 0, buffer);
		return descriptors;
	}

	public static List<Mat> descriptorList(Mat descriptors) {
		byte[] buffer = new byte[descriptors.rows() * 32];
		descriptors.get(0, 0, buffer);
		List<Mat> descriptorList = new ArrayList<Mat>();
		for (int i = 0; i < descriptors.rows(); i++) {
			byte[] bufferTemp = new byte[32];
			for (int j = 0; j < 32; j++) {
				bufferTemp[j] = buffer[i * 32 + j];
			}
			Mat descriptor = new Mat(1, 32, CvType.CV_8U);
			descriptor.put(0, 0, bufferTemp);
			descriptorList.add(descriptor);
		}
		return descriptorList;
	}

	public static Matrix4f MatrixToMatrix4f(Matrix matrix) {
		Matrix4f matrix4f = new Matrix4f();
		matrix4f.m00((float) matrix.get(0, 0));
		matrix4f.m01((float) matrix.get(1, 0));
		matrix4f.m02((float) matrix.get(2, 0));
		matrix4f.m03((float) matrix.get(3, 0));
		matrix4f.m10((float) matrix.get(0, 1));
		matrix4f.m11((float) matrix.get(1, 1));
		matrix4f.m12((float) matrix.get(2, 1));
		matrix4f.m13((float) matrix.get(3, 1));
		matrix4f.m20((float) matrix.get(0, 2));
		matrix4f.m21((float) matrix.get(1, 2));
		matrix4f.m22((float) matrix.get(2, 2));
		matrix4f.m23((float) matrix.get(3, 2));
		matrix4f.m30((float) matrix.get(0, 3));
		matrix4f.m31((float) matrix.get(1, 3));
		matrix4f.m32((float) matrix.get(2, 3));
		matrix4f.m33((float) matrix.get(3, 3));
		return matrix4f;
	}

	public static void printMatrix(Matrix4f mat) {
		pl("");
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				p(mat.getRowColumn(i, j) + "\t");
			}
			pl("");
		}
		pl("");
	}

	// quaternion multiplication, assuming column vector of format [qw, qx, qy,
	// qz].transpose() (q1*q2)
	public static Matrix quatMult(Matrix q1, Matrix q2) {

		Matrix t = new Matrix(4, 1);

		double q1w = q1.get(0, 0);
		double q1x = q1.get(1, 0);
		double q1y = q1.get(2, 0);
		double q1z = q1.get(3, 0);

		double q2w = q2.get(0, 0);
		double q2x = q2.get(1, 0);
		double q2y = q2.get(2, 0);
		double q2z = q2.get(3, 0);

		t.set(1, 0, q1x * q2w + q1y * q2z - q1z * q2y + q1w * q2x);
		t.set(2, 0, -q1x * q2z + q1y * q2w + q1z * q2x + q1w * q2y);
		t.set(3, 0, q1x * q2y - q1y * q2x + q1z * q2w + q1w * q2z);
		t.set(0, 0, -q1x * q2x - q1y * q2y - q1z * q2z + q1w * q2w);

		t = t.times(1 / t.normF());

		return t;
	}

	// quaternion multiplication, assuming column vector of format [qw, qx, qy,
	// qz].transpose() (q1*q2) (wihtout normalization)
	public static Matrix quatMultPure(Matrix q1, Matrix q2) {

		Matrix t = new Matrix(4, 1);

		double q1w = q1.get(0, 0);
		double q1x = q1.get(1, 0);
		double q1y = q1.get(2, 0);
		double q1z = q1.get(3, 0);

		double q2w = q2.get(0, 0);
		double q2x = q2.get(1, 0);
		double q2y = q2.get(2, 0);
		double q2z = q2.get(3, 0);

		t.set(1, 0, q1x * q2w + q1y * q2z - q1z * q2y + q1w * q2x);
		t.set(2, 0, -q1x * q2z + q1y * q2w + q1z * q2x + q1w * q2y);
		t.set(3, 0, q1x * q2y - q1y * q2x + q1z * q2w + q1w * q2z);
		t.set(0, 0, -q1x * q2x - q1y * q2y - q1z * q2z + q1w * q2w);

		return t;
	}

	public static List<Matrix> matsToMatrices(List<Mat> mats) {
		List<Matrix> matrices = new ArrayList<Matrix>();

		for (Mat mat : mats) {
			matrices.add(MatToMatrix(mat));
		}

		return matrices;
	}

	public static List<Matrix> matsToMatricesByte(List<Mat> mats) {
		List<Matrix> matrices = new ArrayList<Matrix>();

		for (Mat mat : mats) {
			matrices.add(MatToMatrixByte(mat));
		}

		return matrices;
	}

	public static Matrix MatToMatrix(Mat mat) {
		double[] buffer = new double[mat.rows() * mat.cols()];
		mat.get(0, 0, buffer);
		Matrix matrix = new Matrix(mat.rows(), mat.cols());
		for (int i = 0; i < matrix.getRowDimension(); i++) {
			for (int j = 0; j < matrix.getColumnDimension(); j++) {
				matrix.set(i, j, buffer[i * mat.cols() + j]);
			}
		}

		return matrix;
	}

	public static Matrix MatToMatrixByte(Mat mat) {
		byte[] buffer = new byte[mat.rows() * mat.cols()];
		mat.get(0, 0, buffer);
		Matrix matrix = new Matrix(mat.rows(), mat.cols());
		for (int i = 0; i < matrix.getRowDimension(); i++) {
			for (int j = 0; j < matrix.getColumnDimension(); j++) {
				matrix.set(i, j, buffer[i * mat.cols() + j]);
			}
		}

		return matrix;
	}

	public static Mat MatrixToMat(Matrix matrix) {
		Mat mat = new Mat(matrix.getRowDimension(), matrix.getColumnDimension(), CvType.CV_64F);
		double[] buffer = new double[matrix.getRowDimension() * matrix.getColumnDimension()];
		for (int i = 0; i < matrix.getRowDimension(); i++) {
			for (int j = 0; j < matrix.getColumnDimension(); j++) {
				buffer[i * matrix.getColumnDimension() + j] = matrix.get(i, j);
			}
		}
		return mat;
	}

	public static Pose matrixToPose(Matrix E) {

		DMatrixRMaj R = new DMatrixRMaj(3, 3);
		R.add(0, 0, E.get(0, 0));
		R.add(0, 1, E.get(0, 1));
		R.add(0, 2, E.get(0, 2));
		R.add(1, 0, E.get(1, 0));
		R.add(1, 1, E.get(1, 1));
		R.add(1, 2, E.get(1, 2));
		R.add(2, 0, E.get(2, 0));
		R.add(2, 1, E.get(2, 1));
		R.add(2, 2, E.get(2, 2));

		Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(R, null);
		q.normalize();

		Pose tempPose = new Pose();

		tempPose.setQw(q.w);
		tempPose.setQx(q.x);
		tempPose.setQy(q.y);
		tempPose.setQz(q.z);

		tempPose.setT(E.get(0, 3), E.get(1, 3), E.get(2, 3));

		return tempPose;

	}

	public static <T> List<T> pruneList(List<T> listToPrune, List<Boolean> keepList) {
		List<T> prunedList = new ArrayList<T>();
		for (int i = 0; i < listToPrune.size(); i++) {
			if (keepList.get(i)) {
				prunedList.add(listToPrune.get(i));
			}
		}
		return prunedList;
	}

	public static void basicStats(List<Double> list, Dbl avg, Dbl stdDev, Dbl median) {
		double total = 0.0;
		for (int i = 0; i < list.size(); i++) {
			total += list.get(i);
		}
		avg.setValue(total / list.size());
		double variance = 0;
		for (int i = 0; i < list.size(); i++) {
			variance += Math.pow(list.get(i) - avg.getValue(), 2);
		}
		variance /= list.size();
		stdDev.setValue(Math.sqrt(variance));

		if (median != null) {
			List<Double> listCopy = new ArrayList<Double>(list);
			Collections.sort(listCopy);
			median.setValue(listCopy.get(listCopy.size() / 2));
		}
	}

}
