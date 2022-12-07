package lumoslam;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Size;
import org.opencv.features2d.FastFeatureDetector;
import org.opencv.features2d.ORB;
import org.opencv.imgproc.Imgproc;

import toolbox.Utils;
import types.ImageData;

public class ImageProcessor {

	public static int[] FASTThresholds = { 50, 50, 50, 50 };
	public Mat logitLookUp = new Mat(1, 256, CvType.CV_8U);
	public FastFeatureDetector FAST = FastFeatureDetector.create(35, true, FastFeatureDetector.TYPE_9_16);
	public ORB orb = ORB.create();

	public int featureNumTarget = 100;

	public ImageProcessor() {
		this.init();
	}

	public void init() {
		this.loadLogitLookUp();
		this.orb.setScoreType(ORB.FAST_SCORE);
		this.orb.setScaleFactor(2);
		this.orb.setNLevels(4);
		Utils.pl("ORB num features: " + this.orb.getMaxFeatures());
		Utils.pl("ORB num levels: " + this.orb.getNLevels());
		Utils.pl("ORB FAST threshold: " + this.orb.getFastThreshold());
		Utils.pl("ORB patch size: " + this.orb.getPatchSize());
		Utils.pl("ORB scale factor: " + this.orb.getScaleFactor());
		Utils.pl("ORB score type: " + this.orb.getScoreType());
		Utils.pl("ORB WTA_K: " + this.orb.getWTA_K());
		Utils.pl("FAST: " + ORB.FAST_SCORE);
	}

	public void loadLogitLookUp() {
		byte[] lookUpTableData = new byte[(int) (this.logitLookUp.total() * this.logitLookUp.channels())];
		for (int i = 0; i < this.logitLookUp.cols(); i++) {
			lookUpTableData[i] = (byte) this.logit(i);
		}
		this.logitLookUp.put(0, 0, lookUpTableData);
	}

	public double logit(double p) {
		// normal range of log((p+1) / (256 - p)) is -5.545177444479562 to
		// +5.545177444479562, which is a range of 11.090354888959125 (for p values
		// [0,255]).
		// (log((p + 1) / (256 - p)) + 5.545177444479562) / 11.090354888959125 * 255
		// simplifies to the following output
		return (Math.log((p + 1) / (256 - p)) + 5.545177444479562) * 22.992952214167854;
	}

	public Mat logitTransform(Mat src) {
		Mat dest = new Mat();
		Core.LUT(src, this.logitLookUp, dest);
		return dest;
	}

	public Mat autoContrast(Mat src) {
		Mat dest = new Mat();
		Imgproc.equalizeHist(src, dest);
		return dest;
	}

	public void autoContrast(ImageData imageData) {
		imageData.setImage(this.autoContrast(imageData.getImage()));
	}

	public MatOfKeyPoint getFastFeatures(Mat img) {
		MatOfKeyPoint keypoints = new MatOfKeyPoint();
		this.FAST.detect(img, keypoints);
		return keypoints;
	}

	public Mat downScale(Mat src) {
		Mat dest = new Mat();
		Imgproc.pyrDown(src, dest, new Size((int) (src.cols() / 2), (int) (src.rows() / 2)));
		return dest;
	}

	public Mat upScale(Mat src) {
		Mat dest = new Mat();
		Imgproc.pyrUp(src, dest, new Size((int) (src.cols() * 2), (int) (src.rows() * 2)));
		return dest;
	}

	public Mat getORBDescriptors(Mat src, MatOfKeyPoint keypoints) {
		Mat descriptors = new Mat();
		this.orb.compute(src, keypoints, descriptors);
		return descriptors;
	}

	public void detectAndComputeORB(ImageData imageData) {
		this.orb.detect(imageData.getImage(), imageData.getKeypoints());
		this.orb.compute(imageData.getImage(), imageData.getKeypoints(), imageData.getDescriptors());
	}

}
