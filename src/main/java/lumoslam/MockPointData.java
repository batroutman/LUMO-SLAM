package lumoslam;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.joml.Vector3f;
import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;

import Jama.Matrix;
import runtimevars.Parameters;
import toolbox.Utils;
import types.ImageData;

public class MockPointData {

	protected int HEIGHT = Parameters.<Integer>get("height");
	protected int WIDTH = Parameters.<Integer>get("width");
	protected long MAX_FRAMES = 800;
	protected int NUM_POINTS = 1000;
	protected int START_FRAME = 0;
	protected int SEED = 1;
	protected Matrix K = new Matrix(3, 3);

	protected double CORRUPTION_RATE = 0.20;
	protected double MISS_RATE = 0.00;
	protected double NOISE_RADIUS = 1;
	protected double NOISE_RATE = 0.00;

	// Starting pose parameters
	// NOTE: translations should be negative (-C)
	protected Vector3f initialTranslation = new Vector3f(0f, 0f, 0f);
	protected double initialRotX = 0.0;
	protected double initialRotY = 0.0;
	protected double initialRotZ = 0.0;

	// Amount to update R and t by each frame
	// NOTE: translations should be negative (-C)
	// protected Vector3f translationVelocity = new Vector3f(0.00f, 0.002f,
	// -0.2f);
//	protected Vector3f translationVelocity = new Vector3f(0.7f, 0.0f, 0.0f);
//	protected double rotX = 0.000;
//	protected double rotY = -0.02;
//	protected double rotZ = -0.000;
	protected Vector3f translationVelocity = new Vector3f(-0.0333f, 0.00f, -0.00f);
	protected double rotX = 0.000;
	protected double rotY = 0.00333;
	protected double rotZ = 0.000;

	// List of homogeneous column vectors (4x1) corresponding to world
	// coordinates
	protected ArrayList<Matrix> worldCoordinates = new ArrayList<Matrix>();

	// fake descriptors for the world points
	protected Mat descriptors = null;

	public MockPointData() {
		this.init();
	}

	protected void init() {
		this.initK();
		this.initWorldCoordinates();
	}

	protected void initK() {
		K.set(0, 0, Parameters.<Float>get("fx").doubleValue());
		K.set(0, 1, Parameters.<Float>get("s").doubleValue());
		K.set(0, 2, Parameters.<Float>get("cx").doubleValue());
		K.set(1, 0, 0.0);
		K.set(1, 1, Parameters.<Float>get("fy").doubleValue());
		K.set(1, 2, Parameters.<Float>get("cy").doubleValue());
		K.set(2, 0, 0.0);
		K.set(2, 1, 0.0);
		K.set(2, 2, 1.0);
	}

	protected void initWorldCoordinates() {
		String output = "";
//		double Z_SPAWN_MIN = -100;
//		double Z_SPAWN_MAX = 100;
//		double Y_SPAWN_MIN = -20;
//		double Y_SPAWN_MAX = 20;
//		double X_SPAWN_MIN = -30;
//		double X_SPAWN_MAX = 30;

		double Z_SPAWN_MIN = 5;
		double Z_SPAWN_MAX = 6;
		double Y_SPAWN_MIN = -2;
		double Y_SPAWN_MAX = 2;
		double X_SPAWN_MIN = -10;
		double X_SPAWN_MAX = 10;

		double Z_RANGE = Z_SPAWN_MAX - Z_SPAWN_MIN;
		double Y_RANGE = Y_SPAWN_MAX - Y_SPAWN_MIN;
		double X_RANGE = X_SPAWN_MAX - X_SPAWN_MIN;

		Random random = new Random(this.SEED);
		for (int i = 0; i < this.NUM_POINTS; i++) {
			Matrix point = new Matrix(4, 1);
			point.set(0, 0, random.nextDouble() * X_RANGE + X_SPAWN_MIN);
			point.set(1, 0, random.nextDouble() * Y_RANGE + Y_SPAWN_MIN);
			point.set(2, 0, random.nextDouble() * Z_RANGE + Z_SPAWN_MIN);
			point.set(3, 0, 1);
			this.worldCoordinates.add(point);
			output += point.get(0, 0) + ", " + point.get(1, 0) + ", " + point.get(2, 0) + "\n";
		}
		// System.out.println("true world coords:");
		// System.out.println(output);

		this.loadEnvironment1();

	}

	public void loadEnvironment1() {
		this.worldCoordinates.clear();

		// floor
		this.worldCoordinates.addAll(this.getPointsInPlane(this.SEED, 100, 0, 1, 5, 0, 1, 0, 20, 20, 20, 0.0)); // 0.1

		// back wall
		this.worldCoordinates.addAll(this.getPointsInPlane(this.SEED, 100, 0, -9, 15, 0, 0, 1, 20, 20, 20, 0.00)); // 0.05

		// right wall
		this.worldCoordinates.addAll(this.getPointsInPlane(this.SEED, 100, 10, -9, 5, 1, 0, 0, 20, 20, 20, 0.00)); // 0.05

		// globe
		this.worldCoordinates.addAll(this.getPointsInSphere(this.SEED, 100, 0, 0, 5, 1, 0.00)); // 0.05

		// right painting
		this.worldCoordinates.addAll(this.getPointsInPlane(this.SEED, 100, 9.9, -1, 5, 1, 0, 0, 1, 2, 4, 0)); // 0

		// back right cluster
		this.worldCoordinates.addAll(this.getPointsInSphere(this.SEED, 100, 5, 0, -10, 1, 0.0)); // 0.5

		// back floor mat
		this.worldCoordinates.addAll(this.getPointsInPlane(this.SEED, 100, -10, 1, -5, 0, 1, 0, 5, 5, 5, 0.0)); // 0.1

		// back left cluster
		this.worldCoordinates.addAll(this.getPointsInSphere(this.SEED, 100, -5, -2, -20, 5, 0.0)); // 0.5
	}

	public List<Matrix> getPointsInPlane(int seed, int numPoints, double x0, double y0, double z0, double normalX,
			double normalY, double normalZ, double xRange, double yRange, double zRange, double noiseRange) {

		List<Matrix> points = new ArrayList<Matrix>();

		// calculate minimum values for each dimension
		double xMin = x0 - xRange / 2;
		double yMin = y0 - yRange / 2;
		double zMin = z0 - zRange / 2;

		Random rand = new Random(seed);

		// generate points
		for (int i = 0; i < numPoints; i++) {

			double x = rand.nextDouble() * xRange + xMin;
			double y = rand.nextDouble() * yRange + yMin;
			double z = rand.nextDouble() * zRange + zMin;

			// correct one of the coordinates
			if (normalZ != 0) {
				z = (-normalX * (x - x0) - normalY * (y - y0)) / normalZ + z0;
			} else if (normalX != 0) {
				x = (-normalZ * (z - z0) - normalY * (y - y0)) / normalX + x0;
			} else if (normalY != 0) {
				y = (-normalZ * (z - z0) - normalX * (x - x0)) / normalY + y0;
			}

			double noiseX = rand.nextDouble() * noiseRange - noiseRange / 2;
			double noiseY = rand.nextDouble() * noiseRange - noiseRange / 2;
			double noiseZ = rand.nextDouble() * noiseRange - noiseRange / 2;

			x += noiseX;
			y += noiseY;
			z += noiseZ;

			Matrix p = new Matrix(4, 1);
			p.set(0, 0, x);
			p.set(1, 0, y);
			p.set(2, 0, z);
			p.set(3, 0, 1);
			points.add(p);

		}

		return points;

	}

	public List<Matrix> getPointsInSphere(int seed, int numPoints, double x0, double y0, double z0, double radius,
			double noiseRange) {

		List<Matrix> points = new ArrayList<Matrix>();

		Random rand = new Random(seed);

		for (int i = 0; i < numPoints; i++) {
			double theta = rand.nextDouble() * 2 * Math.PI;
			double phi = rand.nextDouble() * Math.PI;

			double x = radius * Math.cos(theta) * Math.sin(phi) + x0;
			double y = radius * Math.sin(theta) * Math.sin(phi) + y0;
			double z = radius * Math.cos(phi) + z0;

			double noiseX = rand.nextDouble() * noiseRange - noiseRange / 2;
			double noiseY = rand.nextDouble() * noiseRange - noiseRange / 2;
			double noiseZ = rand.nextDouble() * noiseRange - noiseRange / 2;

			x += noiseX;
			y += noiseY;
			z += noiseZ;

			Matrix p = new Matrix(4, 1);
			p.set(0, 0, x);
			p.set(1, 0, y);
			p.set(2, 0, z);
			p.set(3, 0, 1);
			points.add(p);
		}

		return points;

	}

	// Returns homogeneous 4x4 matrix of JUST rotation parameters
	public Matrix getR(long frameNumber) {

		// Calculate this frame's rotation parameters
		float gamma = (float) -(this.initialRotX + this.rotX * (frameNumber + this.START_FRAME));
		float beta = (float) -(this.initialRotY + this.rotY * (frameNumber + this.START_FRAME));
		float alpha = (float) -(this.initialRotZ + this.rotZ * (frameNumber + this.START_FRAME));

		Matrix Rx = Matrix.identity(4, 4);
		Rx.set(1, 1, Math.cos(gamma));
		Rx.set(2, 2, Math.cos(gamma));
		Rx.set(1, 2, -Math.sin(gamma));
		Rx.set(2, 1, Math.sin(gamma));

		Matrix Ry = Matrix.identity(4, 4);
		Ry.set(0, 0, Math.cos(beta));
		Ry.set(2, 2, Math.cos(beta));
		Ry.set(2, 0, -Math.sin(beta));
		Ry.set(0, 2, Math.sin(beta));

		Matrix Rz = Matrix.identity(4, 4);
		Rz.set(0, 0, Math.cos(alpha));
		Rz.set(1, 1, Math.cos(alpha));
		Rz.set(0, 1, -Math.sin(alpha));
		Rz.set(1, 0, Math.sin(alpha));

		return Rz.times(Ry).times(Rx);
	}

	// Returns homogeneous 4x4 matrix of WORLD translation parameters (this uses
	// negative C)
	public Matrix getIC(long frameNumber) {
		// Calculate C
		Matrix C = Matrix.identity(4, 4);

		C.set(0, 3, -(this.initialTranslation.x + this.translationVelocity.x * (frameNumber + this.START_FRAME)));
		C.set(1, 3, -(this.initialTranslation.y + this.translationVelocity.y * (frameNumber + this.START_FRAME)));
		C.set(2, 3, -(this.initialTranslation.z + this.translationVelocity.z * (frameNumber + this.START_FRAME)));

		return C;
	}

	public Matrix getQuaternion(long frameNumber) {

		// Calculate this frame's rotation parameters
		float gamma = (float) -(this.initialRotX + this.rotX * (frameNumber + this.START_FRAME));
		float beta = (float) -(this.initialRotY + this.rotY * (frameNumber + this.START_FRAME));
		float alpha = (float) -(this.initialRotZ + this.rotZ * (frameNumber + this.START_FRAME));

		double cx = Math.cos(gamma * 0.5);
		double cy = Math.cos(beta * 0.5);
		double cz = Math.cos(alpha * 0.5);
		double sx = Math.sin(gamma * 0.5);
		double sy = Math.sin(beta * 0.5);
		double sz = Math.sin(alpha * 0.5);

		double qw = cx * cy * cz + sx * sy * sz;
		double qx = sx * cy * cz - cx * sy * sz;
		double qy = cx * sy * cz + sx * cy * sz;
		double qz = cx * cy * sz - sx * sy * cz;

		Matrix q = new Matrix(4, 1);
		q.set(0, 0, qw);
		q.set(1, 0, qx);
		q.set(2, 0, qy);
		q.set(3, 0, qz);

		q = q.times(1 / q.normF());

		return q;

	}

	public byte[] getImageBufferRGB(long frameNum) {

		Random rand = new Random(this.SEED + frameNum);
		Random randNoiseRate = new Random(this.SEED);

		byte[] buffer = new byte[this.WIDTH * this.HEIGHT * 3];

		// for each world coordinate, project onto image and add to buffer (if it should
		// be one the image)
		List<Matrix> projectedUsed = new ArrayList<Matrix>();
		for (int i = 0; i < this.worldCoordinates.size(); i++) {
			Matrix projCoord = this.K.times(this.getR(frameNum).times(this.getIC(frameNum)).getMatrix(0, 2, 0, 3))
					.times(this.worldCoordinates.get(i));
			if (projCoord.get(2, 0) < 0) {
				continue;
			}
			projCoord = projCoord.times(1 / projCoord.get(2, 0));
			int x = (int) projCoord.get(0, 0);
			int y = (int) projCoord.get(1, 0);

			// add artificial noise
			if (randNoiseRate.nextDouble() < this.NOISE_RATE) {
				x += rand.nextDouble() * 2 * this.NOISE_RADIUS - this.NOISE_RADIUS;
				y += rand.nextDouble() * 2 * this.NOISE_RADIUS - this.NOISE_RADIUS;
				projCoord.set(0, 0, x);
				projCoord.set(1, 0, y);
			}

			if (x >= 0 && y >= 0 && x < this.WIDTH && y < this.HEIGHT) {
				buffer[3 * (y * this.WIDTH + x)] = (byte) 255;
				buffer[3 * (y * this.WIDTH + x) + 1] = (byte) 255;
				buffer[3 * (y * this.WIDTH + x) + 2] = (byte) 255;
				projectedUsed.add(projCoord);
			}
		}

		// denote points that are purposely corrupted or ignored
		List<Integer> corrupted = this.affectedIndices(new Random(this.SEED + frameNum), this.CORRUPTION_RATE,
				projectedUsed.size());
		List<Integer> missed = this.affectedIndices(new Random(this.SEED + frameNum), this.MISS_RATE,
				projectedUsed.size());

		for (int i = 0; i < corrupted.size(); i++) {
			int x = (int) projectedUsed.get(corrupted.get(i)).get(0, 0);
			int y = (int) projectedUsed.get(corrupted.get(i)).get(1, 0);
			buffer[3 * (y * this.WIDTH + x)] = (byte) 255;
			buffer[3 * (y * this.WIDTH + x) + 1] = (byte) 0;
			buffer[3 * (y * this.WIDTH + x) + 2] = (byte) 0;
		}

		for (int i = 0; i < missed.size(); i++) {
			int x = (int) projectedUsed.get(missed.get(i)).get(0, 0);
			int y = (int) projectedUsed.get(missed.get(i)).get(1, 0);
			buffer[3 * (y * this.WIDTH + x)] = (byte) 0;
			buffer[3 * (y * this.WIDTH + x) + 1] = (byte) 0;
			buffer[3 * (y * this.WIDTH + x) + 2] = (byte) 255;
		}

		return buffer;
	}

	public byte[] getImageBufferGrey(long frameNum) {

		Random rand = new Random(this.SEED + frameNum);
		Random randNoiseRate = new Random(this.SEED);

		byte[] buffer = new byte[this.WIDTH * this.HEIGHT];

		// for each world coordinate, project onto image and add to buffer (if it should
		// be one the image)
		List<Matrix> projectedUsed = new ArrayList<Matrix>();
		for (int i = 0; i < this.worldCoordinates.size(); i++) {
			Matrix projCoord = this.K.times(this.getR(frameNum).times(this.getIC(frameNum)).getMatrix(0, 2, 0, 3))
					.times(this.worldCoordinates.get(i));
			if (projCoord.get(2, 0) < 0) {
				continue;
			}
			projCoord = projCoord.times(1 / projCoord.get(2, 0));
			int x = (int) projCoord.get(0, 0);
			int y = (int) projCoord.get(1, 0);

			// add artificial noise
			if (randNoiseRate.nextDouble() < this.NOISE_RATE) {
				x += rand.nextDouble() * 2 * this.NOISE_RADIUS - this.NOISE_RADIUS;
				y += rand.nextDouble() * 2 * this.NOISE_RADIUS - this.NOISE_RADIUS;
				projCoord.set(0, 0, x);
				projCoord.set(1, 0, y);
			}

			if (x >= 0 && y >= 0 && x < this.WIDTH && y < this.HEIGHT) {
				buffer[y * this.WIDTH + x] = (byte) 255;
				projectedUsed.add(projCoord);
			}
		}

		Utils.pl("projectedUsed.size(): " + projectedUsed.size());

		// denote points that are purposely corrupted or ignored
		List<Integer> corrupted = this.affectedIndices(new Random(this.SEED + frameNum), this.CORRUPTION_RATE,
				projectedUsed.size());
		List<Integer> missed = this.affectedIndices(new Random(this.SEED + frameNum), this.MISS_RATE,
				projectedUsed.size());

		for (int i = 0; i < corrupted.size(); i++) {
			int x = (int) projectedUsed.get(corrupted.get(i)).get(0, 0);
			int y = (int) projectedUsed.get(corrupted.get(i)).get(1, 0);
			buffer[y * this.WIDTH + x] = (byte) 64;
		}

		for (int i = 0; i < missed.size(); i++) {
			int x = (int) projectedUsed.get(missed.get(i)).get(0, 0);
			int y = (int) projectedUsed.get(missed.get(i)).get(1, 0);
			buffer[y * this.WIDTH + x] = (byte) 64;
		}

		return buffer;
	}

	public ImageData getImageData(long frameNum) {

		ImageData imgData = new ImageData();

		Random rand = new Random(this.SEED + frameNum);
		Random randNoiseRate = new Random(this.SEED);

		// for every 3D point, project it onto the camera to get a keypoint and an index
		// (which will be converted into a descriptor
		List<Integer> indices = new ArrayList<Integer>();
		List<KeyPoint> keypoints = new ArrayList<KeyPoint>();
		for (int i = 0; i < this.worldCoordinates.size(); i++) {

			Matrix projCoord = this.K.times(this.getR(frameNum).times(this.getIC(frameNum)).getMatrix(0, 2, 0, 3))
					.times(this.worldCoordinates.get(i));

			// if behind image, skip
			if (projCoord.get(2, 0) < 0) {
				continue;
			}
			projCoord = projCoord.times(1 / projCoord.get(2, 0));
			int x = (int) projCoord.get(0, 0);
			int y = (int) projCoord.get(1, 0);

			// add artificial noise
			if (randNoiseRate.nextDouble() < this.NOISE_RATE) {
				x += rand.nextDouble() * 2 * this.NOISE_RADIUS - this.NOISE_RADIUS;
				y += rand.nextDouble() * 2 * this.NOISE_RADIUS - this.NOISE_RADIUS;
			}

			// if the point is within bounds of the image, construct keypoint and record
			// index
			if (x >= 0 && y >= 0 && x < this.WIDTH && y < this.HEIGHT) {
				Point pt = new Point(x, y);
				KeyPoint kp = new KeyPoint();
				kp.pt = pt;
				keypoints.add(kp);
				indices.add(i);
			}

		}

		// set keypoint mat
		MatOfKeyPoint keypointMat = new MatOfKeyPoint();
		keypointMat.fromList(keypoints);
		imgData.setKeypoints(keypointMat);

		// get and set descriptors
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		Utils.pl("indices.size(): " + indices.size());

		List<Integer> affectedCorrupted = this.affectedIndices(new Random(this.SEED + frameNum), this.CORRUPTION_RATE,
				indices.size());
		Utils.pl("affectedCorrupted.size(): " + affectedCorrupted.size());
		this.corruptIndices(indices, affectedCorrupted);

		imgData.setDescriptors(this.generateDescriptors(indices));

		// throw out random features
		List<Integer> affectedMissed = this.affectedIndices(new Random(this.SEED + frameNum), this.MISS_RATE,
				indices.size());
		this.missFeatures(frameNum, imgData, affectedMissed);

		return imgData;

	}

	// given a random number generator, a probability, and a list size, return a new
	// list (different size) of the indices to alter given this probability.
	public List<Integer> affectedIndices(Random rand, double probability, int size) {
		List<Integer> indices = new ArrayList<Integer>();
		for (int i = 0; i < size; i++) {
			double roll = rand.nextDouble();
			if (roll < probability) {
				indices.add(i);
			}
		}
		return indices;
	}

	// randomly throw out a proportion of the keypoints and descriptors to test
	// robustness
	public void missFeatures(long frameNum, ImageData imgData, List<Integer> missIndices) {

		List<KeyPoint> keypoints = new ArrayList<KeyPoint>();
		List<Mat> descriptors = new ArrayList<Mat>();
		keypoints.addAll(imgData.getKeypoints().toList());
		descriptors.addAll(Utils.descriptorList(imgData.getDescriptors()));

		for (int i = missIndices.size() - 1; i >= 0; i--) {
			keypoints.remove((int) missIndices.get(i));
			descriptors.remove((int) missIndices.get(i));
		}

		imgData.getKeypoints().fromList(keypoints);
		imgData.setDescriptors(Utils.mergeDescriptorList(descriptors));

	}

	// given a small corruption rate (about 0.05 or 0.1), randomly add 1 to the
	// index values at that rate
	// this is to simulate incorrect matches/outliers
	public void corruptIndices(List<Integer> indices, List<Integer> corruptedIndices) {

		for (int i = 0; i < corruptedIndices.size(); i++) {
			indices.set(corruptedIndices.get(i), indices.get(corruptedIndices.get(i)) + 1);
		}

	}

	public Mat generateDescriptors(List<Integer> indices) {
		Mat descriptors = new Mat(indices.size(), 32, CvType.CV_8U);
		byte[] buffer = new byte[indices.size() * 32];
		for (int i = 0; i < indices.size(); i++) {
			int index = indices.get(i);
			byte[] descriptor = new byte[32];
			for (int j = 0; j < 32; j++) {
				byte result = (byte) (index - 255 > 0 ? 255 : index);
				index = index - 255 < 0 ? 0 : index - 255;
				buffer[i * 32 + j] = result;
				descriptor[j] = result;
			}
//			Utils.pa(descriptor);
		}

		descriptors.put(0, 0, buffer);
		return descriptors;
	}

	public ArrayList<Matrix> getWorldCoordinates() {
		return worldCoordinates;
	}

	public void setWorldCoordinates(ArrayList<Matrix> worldCoordinates) {
		this.worldCoordinates = worldCoordinates;
	}

	public int getHEIGHT() {
		return HEIGHT;
	}

	public void setHEIGHT(int hEIGHT) {
		HEIGHT = hEIGHT;
	}

	public int getWIDTH() {
		return WIDTH;
	}

	public void setWIDTH(int wIDTH) {
		WIDTH = wIDTH;
	}

	public long getMAX_FRAMES() {
		return MAX_FRAMES;
	}

	public void setMAX_FRAMES(long mAX_FRAMES) {
		MAX_FRAMES = mAX_FRAMES;
	}

	public int getNUM_POINTS() {
		return NUM_POINTS;
	}

	public void setNUM_POINTS(int nUM_POINTS) {
		NUM_POINTS = nUM_POINTS;
	}

	public Matrix getK() {
		return K;
	}

	public void setK(Matrix k) {
		K = k;
	}

	public Vector3f getInitialTranslation() {
		return initialTranslation;
	}

	public void setInitialTranslation(Vector3f initialTranslation) {
		this.initialTranslation = initialTranslation;
	}

	public double getInitialRotX() {
		return initialRotX;
	}

	public void setInitialRotX(double initialRotX) {
		this.initialRotX = initialRotX;
	}

	public double getInitialRotY() {
		return initialRotY;
	}

	public void setInitialRotY(double initialRotY) {
		this.initialRotY = initialRotY;
	}

	public double getInitialRotZ() {
		return initialRotZ;
	}

	public void setInitialRotZ(double initialRotZ) {
		this.initialRotZ = initialRotZ;
	}

	public Vector3f getTranslationVelocity() {
		return translationVelocity;
	}

	public void setTranslationVelocity(Vector3f translationVelocity) {
		this.translationVelocity = translationVelocity;
	}

	public double getRotX() {
		return rotX;
	}

	public void setRotX(double rotX) {
		this.rotX = rotX;
	}

	public double getRotY() {
		return rotY;
	}

	public void setRotY(double rotY) {
		this.rotY = rotY;
	}

	public double getRotZ() {
		return rotZ;
	}

	public void setRotZ(double rotZ) {
		this.rotZ = rotZ;
	}

	public Mat getDescriptors() {
		return descriptors;
	}

	public void setDescriptors(Mat descriptors) {
		this.descriptors = descriptors;
	}
}
