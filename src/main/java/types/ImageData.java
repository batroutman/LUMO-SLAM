package types;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.features2d.ORB;
import org.opencv.imgproc.Imgproc;

import toolbox.Utils;

public class ImageData {

	public static ORB orb = ORB.create(1000, 1.2f, 8, 31, 0, 2, ORB.FAST_SCORE, 31, 20); // default-ish

	protected List<Mat> masks = new ArrayList<Mat>();

	protected Mat image = new Mat();
	protected MatOfKeyPoint keypoints = new MatOfKeyPoint();
	protected Mat descriptors = new Mat();
	protected List<Mat> mostRecentDescriptors = new ArrayList<Mat>();

	public ImageData() {
	}

	public ImageData(Mat image) {
		this.image = image;
	}

	public byte[] getBuffer() {
		byte[] buffer = new byte[this.image.rows() * this.image.cols()];
		this.image.get(0, 0, buffer);
		return buffer;
	}

	public void autoContrast() {
		Imgproc.equalizeHist(this.image, this.image);
	}

	public void computeFeatures() {
		orb.compute(this.image, this.keypoints, this.descriptors);
		this.mostRecentDescriptors = Utils.descriptorList(this.descriptors);
	}

	public Mat getImage() {
		return image;
	}

	public void setImage(Mat image) {
		this.image = image;
	}

	public MatOfKeyPoint getKeypoints() {
		return keypoints;
	}

	public void setKeypoints(MatOfKeyPoint keypoints) {
		this.keypoints = keypoints;
	}

	public Mat getDescriptors() {
		return descriptors;
	}

	public void setDescriptors(Mat descriptors) {
		this.descriptors = descriptors;
	}

	public List<Mat> getMostRecentDescriptors() {
		return mostRecentDescriptors;
	}

	public void setMostRecentDescriptors(List<Mat> mostRecentDescriptors) {
		this.mostRecentDescriptors = mostRecentDescriptors;
	}

}
