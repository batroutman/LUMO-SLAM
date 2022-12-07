package mock;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;

import Jama.Matrix;
import runtimevars.Parameters;
import toolbox.Utils;
import types.ImageData;
import types.Transformation;

public class MockEnvironment {

	public long totalFrames = 115;
	List<PointCloudObject> objects = new ArrayList<PointCloudObject>();
	MovingObject cameraMotion = new MovingObject();
	Matrix K = new Matrix(3, 3);

	public MockEnvironment() {
		this.initK();
		initCameraMotion2();
		initEnvironment2();
	}

	public void initK() {
		K.set(0, 0, Parameters.<Float>get("fx").doubleValue());
		K.set(0, 1, Parameters.<Float>get("s").doubleValue());
		K.set(0, 2, Parameters.<Float>get("cx").doubleValue());
		K.set(1, 1, Parameters.<Float>get("fy").doubleValue());
		K.set(1, 2, Parameters.<Float>get("cy").doubleValue());
		K.set(2, 2, 1);
	}

	public void initCameraMotion1() {
		double velocity = 0.005;
		double dist = 1;
		double rot = Math.acos((2 * dist * dist - velocity * velocity) / (2 * dist * dist)); // law of cosines
		MotionModel right = new LinearMotionModel(0, 0, 0, -velocity, 0, 0, LinearMotionModel.TranslationType.RELATIVE);
		this.cameraMotion.addMotionModel(0L, right);
	}

	public void initCameraMotion2() {
		double velocity = 0.005;
		StaticMotionModel tilt = new StaticMotionModel();
		tilt.getTransformation().setCx(0.5);
		tilt.getTransformation().setCz(-0.9);
		tilt.getTransformation().rotateEuler(0.2, 0, 0);
		MotionModel right = new LinearMotionModel(0, 0, 0, -velocity, 0, 0, LinearMotionModel.TranslationType.RELATIVE);
		this.cameraMotion.addMotionModel(0L, tilt);
		this.cameraMotion.addMotionModel(1L, right);
	}

	public void initEnvironment1() {
		PointCloudObject staticPoints = new PointCloudObject();
		staticPoints.addPlane(0, 500, 0, 0.5, 0, 0, -1, 0, 10, 1, 10, 0); // floor
		staticPoints.addPlane(0, 100, 1.5, 0, 3, 0, 0, 1, 1, 1, 1, 0.3); // back wall

		PointCloudObject movingObject = new PointCloudObject();
		movingObject.addSphere(0, 20, 0.5, 0, 2, 0.1, 0);
		movingObject.addMotionModel(200L,
				new LinearMotionModel(0, 0, 0, 0, -0.01, 0, LinearMotionModel.TranslationType.ABSOLUTE));

		this.objects.add(staticPoints);
		this.objects.add(movingObject);
	}

	public void initEnvironment2() {
		PointCloudObject staticPoints = new PointCloudObject();
		staticPoints.addPlane(0, 100, 0, 1, 1, 0, -1, 0, 10, 0, 10, 0); // floor
		staticPoints.addPlane(0, 40, 0, 0.75, 2, 0, 0, 1, 1, 0.5, 0, 0); // desk front
		staticPoints.addPlane(0, 40, 0.5, 0.75, 2.5, 1, 0, 0, 0, 0.5, 1, 0); // desk right
		staticPoints.addPlane(0, 40, -0.5, 0.75, 2.5, 1, 0, 0, 0, 0.5, 1, 0); // desk left
		staticPoints.addPlane(0, 40, 0, 0.5, 2.25, 0, -1, 0, 1, 0, 1, 0); // desk top

		PointCloudObject movingObject = new PointCloudObject();
		movingObject.addSphere(0, 20, 0, 0.25, 2.25, 0.1, 0);
		movingObject.addMotionModel(100L,
				new LinearMotionModel(0, 0, 0, 0, -0.01, 0, LinearMotionModel.TranslationType.ABSOLUTE));

		PointCloudObject movingObject2 = new PointCloudObject();
		movingObject2.addSphere(0, 20, 0.2, 0.25, 2.25, 0.1, 0);
		movingObject2.addMotionModel(110L,
				new LinearMotionModel(0, 0, 0, -0.01, 0, 0, LinearMotionModel.TranslationType.ABSOLUTE));

		this.objects.add(staticPoints);
		this.objects.add(movingObject);
		this.objects.add(movingObject2);
	}

	// create image data for frameNum, keypoints, and descriptors
	public ImageData getImageData(long frameNum, List<Byte> outImageBufferGrey, List<Byte> outImageBufferRGB) {

		long start = System.currentTimeMillis();

		// get all transformed points for the frame
		List<Matrix> allPointsTransformed = new ArrayList<Matrix>();
		for (int i = 0; i < this.objects.size(); i++) {
			allPointsTransformed.addAll(this.objects.get(i).getTransformedPoints(frameNum));
		}

		// create mapping from point to index in this list of all points
		HashMap<Matrix, Integer> pointToIndex = new HashMap<Matrix, Integer>();
		for (int i = 0; i < allPointsTransformed.size(); i++) {
			pointToIndex.put(allPointsTransformed.get(i), i);
		}

		// for each point, project it into frame and register descriptors and keypoints
		// for points that fall on the image
		List<KeyPoint> keypointsList = new ArrayList<KeyPoint>();
		List<Integer> indices = new ArrayList<Integer>();
		Transformation cameraTransformation = this.cameraMotion.getTransformation(frameNum);
		Matrix cameraMatrix = cameraTransformation.getHomogeneousMatrix().getMatrix(0, 2, 0, 3);
		Matrix P = this.K.times(cameraMatrix).getMatrix(0, 2, 0, 3);

		for (int i = 0; i < allPointsTransformed.size(); i++) {

			Matrix proj = P.times(allPointsTransformed.get(i));
			double w = proj.get(2, 0);

			// if behind camera, skip
			if (w <= 0) {
				continue;
			}

			proj = proj.times(1 / w);

			// if off-screen, skip
			double x = proj.get(0, 0);
			double y = proj.get(1, 0);
			if (x < 0 || y < 0 || x > Parameters.<Integer>get("width") || y > Parameters.<Integer>get("height")) {
				continue;
			}

			Point pt = new Point(x, y);
			KeyPoint kp = new KeyPoint();
			kp.pt = pt;
			keypointsList.add(kp);
			indices.add(i);

		}

		// corrupt indices for robustness testing (TODO)

		// generate descriptor Mat
		Mat descriptors = this.generateDescriptors(indices);
		List<Mat> descriptorsList = Utils.descriptorList(descriptors);

		// with the resulting keypoints, generate image buffers (function calls)
		this.generateImageBufferGrey(keypointsList, outImageBufferGrey);
		this.generateImageBufferRGB(keypointsList, outImageBufferRGB);

		// pack into ImageData and return
		ImageData imgData = new ImageData();
		imgData.setDescriptors(descriptors);
		imgData.setMostRecentDescriptors(descriptorsList);
		MatOfKeyPoint keypointMat = new MatOfKeyPoint();
		keypointMat.fromList(keypointsList);
		imgData.setKeypoints(keypointMat);

		long end = System.currentTimeMillis();
		Utils.pl("mock image computation: " + (end - start) + "ms");

		return imgData;

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

	public void generateImageBufferGrey(List<KeyPoint> keypoints, List<Byte> outImageBufferGrey) {

		byte bg = (byte) 0;
		byte fg = (byte) 255;

		// init buffer
		outImageBufferGrey.clear();
		for (int i = 0; i < Parameters.<Integer>get("width"); i++) {
			for (int j = 0; j < Parameters.<Integer>get("height"); j++) {
				outImageBufferGrey.add(bg);
			}
		}

		// for each keypoint, set the corresponding foreground pixel
		for (KeyPoint kp : keypoints) {
			int x = (int) kp.pt.x;
			int y = (int) kp.pt.y;
			outImageBufferGrey.set(x + y * Parameters.<Integer>get("width"), fg);
		}

	}

	public void generateImageBufferRGB(List<KeyPoint> keypoints, List<Byte> outImageBufferRGB) {

		byte bgR = (byte) 0;
		byte bgG = (byte) 0;
		byte bgB = (byte) 0;

		byte fgR = (byte) 0;
		byte fgG = (byte) 255;
		byte fgB = (byte) 0;

		// init buffer
		outImageBufferRGB.clear();
		for (int i = 0; i < Parameters.<Integer>get("width"); i++) {
			for (int j = 0; j < Parameters.<Integer>get("height"); j++) {
				outImageBufferRGB.add(bgR);
				outImageBufferRGB.add(bgG);
				outImageBufferRGB.add(bgB);
			}
		}

		// for each keypoint, set the corresponding foreground pixel
		for (KeyPoint kp : keypoints) {
			int x = (int) kp.pt.x;
			int y = (int) kp.pt.y;
			outImageBufferRGB.set(3 * (x + y * Parameters.<Integer>get("width")) + 0, fgR);
			outImageBufferRGB.set(3 * (x + y * Parameters.<Integer>get("width")) + 1, fgG);
			outImageBufferRGB.set(3 * (x + y * Parameters.<Integer>get("width")) + 2, fgB);
		}

	}

}
