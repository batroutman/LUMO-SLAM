package types;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Size;
import org.opencv.features2d.FastFeatureDetector;
import org.opencv.features2d.ORB;
import org.opencv.imgproc.Imgproc;

import toolbox.Utils;

public class ImageData {

//	public static ORB orb = ORB.create(1000, 2, 3, 31, 0, 2, ORB.FAST_SCORE, 31, 20); // optimized for speed
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

	public void detectAndComputeORB() {
		orb.detect(this.image, this.keypoints);
		orb.compute(this.image, this.keypoints, this.descriptors);
		this.mostRecentDescriptors = Utils.descriptorList(this.descriptors);
	}

	public void computeFeatures() {
		orb.compute(this.image, this.keypoints, this.descriptors);
		this.mostRecentDescriptors = Utils.descriptorList(this.descriptors);
	}

	public void detectAndComputeHomogeneousORB() {
		this.detectHomogeneousFeatures();
		orb.compute(this.image, this.keypoints, this.descriptors);
	}

	public void detectHomogeneousFeatures() {

		// downscale the image to extract FAST features with patch size 28, 56, and 112
		Mat down28 = this.downScale(this.image);
		down28 = this.downScale(down28);
		Mat down56 = this.downScale(down28);
		Mat down112 = this.downScale(down56);

		// init FAST
		int fastThresh = 20;
		FastFeatureDetector fastDetector = FastFeatureDetector.create(fastThresh);

		// get FAST features
		MatOfKeyPoint keypoints28 = new MatOfKeyPoint();
		MatOfKeyPoint keypoints56 = new MatOfKeyPoint();
		MatOfKeyPoint keypoints112 = new MatOfKeyPoint();
		fastDetector.detect(down28, keypoints28);
		fastDetector.detect(down56, keypoints56);
		fastDetector.detect(down112, keypoints112);

		// sort lists
		List<KeyPoint> sorted28 = keypoints28.toList();
		List<KeyPoint> sorted56 = keypoints56.toList();
		List<KeyPoint> sorted112 = keypoints112.toList();
		sorted28.sort((kp1, kp2) -> (int) (kp2.response - kp1.response));
		sorted56.sort((kp1, kp2) -> (int) (kp2.response - kp1.response));
		sorted112.sort((kp1, kp2) -> (int) (kp2.response - kp1.response));

		// perform ssc filtering (https://github.com/BAILOOL/ANMS-Codes)
		int numRetPoints28 = 400;
		int numRetPoints56 = 200;
		int numRetPoints112 = 100;
		float tolerance = (float) 0.5;

		List<KeyPoint> sscKeyPoints28 = new ArrayList<KeyPoint>();
		sscKeyPoints28.addAll(sorted28);
		sscKeyPoints28.addAll(sorted56);
		sscKeyPoints28.addAll(sorted112);
//		List<KeyPoint> sscKeyPoints28 = ssc(sorted28, numRetPoints28, tolerance, down28.cols(), down28.rows());
//		List<KeyPoint> sscKeyPoints56 = ssc(sorted56, numRetPoints56, tolerance, down56.cols(), down56.rows());
//		List<KeyPoint> sscKeyPoints112 = ssc(sorted112, numRetPoints112, tolerance, down112.cols(), down112.rows());

		// scale keypoints up to match the full sized image
		for (int i = 0; i < sorted28.size(); i++) {
			KeyPoint keypoint = sorted28.get(i);
			keypoint.size = 31;
			keypoint.octave = 0;
			keypoint.pt.x *= 4;
			keypoint.pt.y *= 4;
		}

		for (int i = 0; i < sorted56.size(); i++) {
			KeyPoint keypoint = sorted56.get(i);
			keypoint.size = 62;
			keypoint.octave = 1;
			keypoint.pt.x *= 8;
			keypoint.pt.y *= 8;
		}

		for (int i = 0; i < sorted112.size(); i++) {
			KeyPoint keypoint = sorted112.get(i);
			keypoint.size = 124;
			keypoint.octave = 2;
			keypoint.pt.x *= 16;
			keypoint.pt.y *= 16;
		}

		// merge keypoints
//		sscKeyPoints28.addAll(sscKeyPoints56);
//		sscKeyPoints28.addAll(sscKeyPoints112);

		int[] patchSizes = { 31, 62, 124 };
		HashMap<Integer, List<Integer>> u_max_map = this.getUMaxMap(patchSizes);
		byte[] imgBuffer = new byte[this.image.rows() * this.image.cols()];
		this.image.get(0, 0, imgBuffer);

		long start = System.currentTimeMillis();
		this.ICAngles(imgBuffer, this.image.cols(), this.image.rows(), sscKeyPoints28, u_max_map);
		long end = System.currentTimeMillis();
		Utils.pl("ICAngle time: " + (end - start) + "ms");

		Utils.pl("sscKeyPoints28.size(): " + sscKeyPoints28.size());
		this.keypoints.fromList(sscKeyPoints28);

	}

	// pass in the full sized image buffer, width of the image, keypoints (with
	// corrected xy values and sizes)
	public void ICAngles(byte[] imgBuffer, int imgWidth, int imgHeight, List<KeyPoint> pts,
			HashMap<Integer, List<Integer>> u_max_map) {
		int ptidx, ptsize = pts.size();
		int width = imgWidth;
		int height = imgHeight;

		for (ptidx = 0; ptidx < ptsize; ptidx++) {

			List<Integer> u_max = u_max_map.get((int) pts.get(ptidx).size);

			int half_k = (int) (pts.get(ptidx).size / 2);

			int centerX = (int) pts.get(ptidx).pt.x;
			int centerY = (int) pts.get(ptidx).pt.y;

			int m_01 = 0, m_10 = 0;

			// Treat the center line differently, v=0
			for (int u = -half_k; u <= half_k; ++u) {
				int x = u + centerX;
				m_10 += x < 0 || x >= width ? 0 : u * imgBuffer[rowMajor(x, centerY, width)];
			}

			// Go line by line in the circular patch
			for (int v = 1; v <= half_k; ++v) {
				// Proceed over the two lines
				int v_sum = 0;
				int d = u_max.get(v);
				for (int u = -d; u <= d; ++u) {
					int x = centerX + u;
					int y = centerY + v;

					int val_plus = x < 0 || y < 0 || x >= width || y >= height ? 0 : imgBuffer[rowMajor(x, y, width)];
					y = centerY - v;
					int val_minus = x < 0 || y < 0 || x >= width || y >= height ? 0 : imgBuffer[rowMajor(x, y, width)];

					v_sum += (val_plus - val_minus);
					m_10 += u * (val_plus + val_minus);
				}
				m_01 += v * v_sum;
			}

			pts.get(ptidx).angle = (float) Core.fastAtan2((float) m_01, (float) m_10);
		}
	}

	public HashMap<Integer, List<Integer>> getUMaxMap(int[] patchSizes) {

		HashMap<Integer, List<Integer>> u_max_map = new HashMap<Integer, List<Integer>>();

		for (int i = 0; i < patchSizes.length; i++) {

			int halfPatchSize = patchSizes[i] / 2;
			List<Integer> umax = new ArrayList<Integer>(halfPatchSize + 2);

			for (int j = 0; j < halfPatchSize + 2; j++) {
				umax.add(0);
			}

			int v, v0, vmax = (int) Math.floor(halfPatchSize * Math.sqrt(2.f) / 2 + 1);
			int vmin = (int) Math.ceil(halfPatchSize * Math.sqrt(2.f) / 2);
			for (v = 0; v <= vmax; ++v)
				umax.set(v, (int) Math.round(Math.sqrt((double) halfPatchSize * halfPatchSize - v * v)));

			// Make sure we are symmetric
			for (v = halfPatchSize, v0 = 0; v >= vmin; --v) {
				while (umax.get(v0) == umax.get(v0 + 1))
					++v0;
				umax.set(v, v0);
				++v0;
			}

			u_max_map.put(patchSizes[i], umax);

		}

		return u_max_map;

	}

	public static int rowMajor(int x, int y, int width) {
		return width * y + x;
	}

	public Mat downScale(Mat src) {
		Mat dest = new Mat();
		Imgproc.pyrDown(src, dest, new Size((int) (src.cols() / 2), (int) (src.rows() / 2)));
		return dest;
	}

	public void filterKeypoints() {
		int numRetPoints = 300;
		float tolerance = 0.1f;
		int cols = this.image.cols();
		int rows = this.image.rows();
		List<KeyPoint> filteredKeypoints = ssc(this.keypoints.toList(), numRetPoints, tolerance, cols, rows);
		this.keypoints.fromList(filteredKeypoints);
	}

	/*
	 * Suppression via Square Convering (SSC) algorithm. Check Algorithm 2 in the
	 * paper:
	 * https://www.sciencedirect.com/science/article/abs/pii/S016786551830062X
	 */
	// https://github.com/BAILOOL/ANMS-Codes
	private static List<KeyPoint> ssc(final List<KeyPoint> keyPoints, final int numRetPoints, final float tolerance,
			final int cols, final int rows) {

		// Several temp expression variables to simplify equation solution
		int expression1 = rows + cols + 2 * numRetPoints;
		long expression2 = ((long) 4 * cols + (long) 4 * numRetPoints + (long) 4 * rows * numRetPoints
				+ (long) rows * rows + (long) cols * cols - (long) 2 * rows * cols
				+ (long) 4 * rows * cols * numRetPoints);
		double expression3 = Math.sqrt(expression2);
		double expression4 = (double) numRetPoints - 1;

		// first solution
		double solution1 = -Math.round((expression1 + expression3) / expression4);
		// second solution
		double solution2 = -Math.round((expression1 - expression3) / expression4);

		// binary search range initialization with positive solution
		int high = (int) ((solution1 > solution2) ? solution1 : solution2);
		int low = (int) Math.floor(Math.sqrt((double) keyPoints.size() / numRetPoints));
		int width;
		int prevWidth = -1;

		ArrayList<Integer> resultVec = new ArrayList<>();
		boolean complete = false;
		int kMin = Math.round(numRetPoints - (numRetPoints * tolerance));
		int kMax = Math.round(numRetPoints + (numRetPoints * tolerance));

		ArrayList<Integer> result = new ArrayList<>(keyPoints.size());
		while (!complete) {
			width = low + (high - low) / 2;
			width = width == 0 ? 1 : width;

			// needed to reassure the same radius is not repeated again
			if (width == prevWidth || low > high) {
				// return the keypoints from the previous iteration
				resultVec = result;
				break;
			}
			result.clear();
			double c = (double) width / 2; // initializing Grid
			int numCellCols = (int) Math.floor(cols / c);
			int numCellRows = (int) Math.floor(rows / c);

			// Fill temporary boolean array
			boolean[][] coveredVec = new boolean[numCellRows + 1][numCellCols + 1];

			// Perform square suppression
			for (int i = 0; i < keyPoints.size(); i++) {
				// get position of the cell current point is located at
				int row = (int) Math.floor(keyPoints.get(i).pt.y / c);
				int col = (int) Math.floor(keyPoints.get(i).pt.x / c);
				if (!coveredVec[row][col]) { // if the cell is not covered
					result.add(i);

					// get range which current radius is covering
					int rowMin = (int) (((row - (int) Math.floor(width / c)) >= 0) ? (row - Math.floor(width / c)) : 0);
					int rowMax = (int) (((row + Math.floor(width / c)) <= numCellRows) ? (row + Math.floor(width / c))
							: numCellRows);
					int colMin = (int) (((col - Math.floor(width / c)) >= 0) ? (col - Math.floor(width / c)) : 0);
					int colMax = (int) (((col + Math.floor(width / c)) <= numCellCols) ? (col + Math.floor(width / c))
							: numCellCols);

					// cover cells within the square bounding box with width w
					for (int rowToCov = rowMin; rowToCov <= rowMax; rowToCov++) {
						for (int colToCov = colMin; colToCov <= colMax; colToCov++) {
							if (!coveredVec[rowToCov][colToCov]) {
								coveredVec[rowToCov][colToCov] = true;
							}
						}
					}
				}
			}

			// solution found
			if (result.size() >= kMin && result.size() <= kMax) {
				resultVec = result;
				complete = true;
			} else if (result.size() < kMin) {
				high = width - 1; // update binary search range
			} else {
				low = width + 1; // update binary search range
			}
			prevWidth = width;
		}

		// Retrieve final keypoints
		List<KeyPoint> kp = new ArrayList<>();
		for (int i : resultVec) {
			kp.add(keyPoints.get(i));
		}

		return kp;
	}

	// get features in grid cells of image (CHANGING MASKS IS UNUSABLY SLOW (333ms))
	public void detectHomogeneousKeypoints() {

		int gridWidth = 10;
		int gridHeight = 10;

		// calculate cell width and height in pixels
		int cellWidth = this.image.cols() / gridWidth;
		int cellHeight = this.image.rows() / gridHeight;

		long start = System.currentTimeMillis();
		// generate masks
		List<Mat> masks = new ArrayList<Mat>();
		for (int row = 0; row < gridHeight; row++) {
			for (int col = 0; col < gridWidth; col++) {
				byte[] buffer = new byte[this.image.rows() * this.image.cols()];
				this.loadZeros(buffer);

				int startX = col * cellWidth;
				int startY = row * cellHeight;
				int endX = startX + cellWidth > this.image.cols() ? this.image.cols() : startX + cellWidth;
				int endY = startY + cellHeight > this.image.rows() ? this.image.rows() : startY + cellHeight;

				this.load255(buffer, startX, startY, endX, endY);
				Mat mask = new Mat(this.image.rows(), this.image.cols(), this.image.type());
				mask.put(0, 0, buffer);
				masks.add(mask);

			}
		}

		long end = System.currentTimeMillis();
		Utils.pl("mask generation time: " + (end - start) + "ms");

		// repeatedly call ORB with each mask to get keypoints
		List<List<KeyPoint>> allKeypoints = new ArrayList<List<KeyPoint>>();
		for (int i = 0; i < masks.size(); i++) {
			Mat mask = masks.get(i);
			MatOfKeyPoint maskedKeypoints = new MatOfKeyPoint();
			this.orb.detect(this.image, maskedKeypoints, mask);
			allKeypoints.add(maskedKeypoints.toList());
		}

		// merge lists and set
		List<KeyPoint> keypoints = new ArrayList<KeyPoint>();
		for (int i = 0; i < allKeypoints.size(); i++) {
			keypoints.addAll(allKeypoints.get(i));
		}

		this.keypoints.fromList(keypoints);

	}

	public void loadZeros(byte[] buffer) {
		for (int i = 0; i < buffer.length; i++) {
			buffer[i] = 0;
		}
	}

	public void load255(byte[] buffer, int startX, int startY, int endX, int endY) {
		for (int x = startX; x < endX; x++) {
			for (int y = startY; y < endY; y++) {
				buffer[this.image.cols() * y + x] = (byte) 255;
			}
		}
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
