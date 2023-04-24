package lumoslam;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Size;
import org.opencv.features2d.FastFeatureDetector;
import org.opencv.imgproc.Imgproc;

import runtimevars.Parameters;
import toolbox.Utils;
import types.ImageData;

public class ORBExtractor {

	public float SCALE_FACTOR = 1.2f;
	public int N_LEVELS = 8;
	public int PATCH_SIZE = 31;

	public static int NORMAL_FAST_THRESHOLD = 60;
	public static int DIRE_FAST_THRESHOLD = 20;

	private ORBExtractor() {

	}

	public ORBExtractor(float scaleFactor, int nLevels, int patchSize) {
		this.SCALE_FACTOR = scaleFactor;
		this.N_LEVELS = nLevels;
		this.PATCH_SIZE = patchSize;
	}

	// extract keypoints such that they are spread throughout the image
	public void setKeypoints(ImageData imageData) {
		// get image pyramid
		List<Mat> pyramid = this.getImagePyramid(imageData.getImage(), this.SCALE_FACTOR, this.N_LEVELS);

		// get all fast features in pyramid
		List<KeyPoint> features = this.getFastFeatures(pyramid, this.SCALE_FACTOR, this.PATCH_SIZE);

		// filter out extraneous features
		List<KeyPoint> filteredFeatures = this.filterFeatures(features);

		// get angles of keypoints
		this.setAngles(filteredFeatures, pyramid, this.PATCH_SIZE);

		MatOfKeyPoint keypoints = new MatOfKeyPoint();
		keypoints.fromList(filteredFeatures);

		imageData.setKeypoints(keypoints);

	}

	public List<Mat> getImagePyramid(Mat image, float scaleFactor, int nLevels) {
		List<Mat> pyramid = new ArrayList<Mat>();
		int imageWidth = image.width();
		int imageHeight = image.height();

		// load pyramid
		long start = System.currentTimeMillis();
		pyramid.add(image.clone());
		for (int i = 1; i < nLevels; i++) {
			double tScaleFactor = Math.pow(scaleFactor, i);
			int width = (int) (imageWidth / tScaleFactor);
			int height = (int) (imageHeight / tScaleFactor);
			Size sz = new Size(width, height);
			Mat resized = new Mat();
			Imgproc.resize(image, resized, sz);
			pyramid.add(resized);
		}
		long end = System.currentTimeMillis();
		Utils.pl("Pyramid time: " + (end - start) + "ms");
		return pyramid;
	}

	public List<KeyPoint> getFastFeatures(List<Mat> pyramid, float scaleFactor, int patchSize) {

		List<KeyPoint> allKeypoints = new ArrayList<KeyPoint>();

		FastFeatureDetector FAST = FastFeatureDetector.create(DIRE_FAST_THRESHOLD, true);

		long start = System.currentTimeMillis();
		for (int level = 0; level < pyramid.size(); level++) {
			Mat image = pyramid.get(level);
			MatOfKeyPoint kpMat = new MatOfKeyPoint();
			FAST.detect(image, kpMat);

			List<KeyPoint> kpList = kpMat.toList();

			float currentSF = 1;
			for (int i = 0; i < level; i++) {
				currentSF *= scaleFactor;
			}

			float size = (float) (patchSize * currentSF);
			for (int k = 0; k < kpList.size(); k++) {
				KeyPoint kp = kpList.get(k);

				// modify keypoint
				kp.octave = level;
				kp.size = size;
				kp.pt.x = kp.pt.x * currentSF;
				kp.pt.y = kp.pt.y * currentSF;

//				if (k == 0) {
//					Utils.pl("");
//					Utils.pl("Keypoint: ");
//					Utils.pl("angle: " + kp.angle);
//					Utils.pl("class_id: " + kp.class_id);
//					Utils.pl("octave: " + kp.octave);
//					Utils.pl("pt.x: " + kp.pt.x);
//					Utils.pl("pt.y: " + kp.pt.y);
//					Utils.pl("response: " + kp.response);
//					Utils.pl("size: " + kp.size);
//				}

			}

			// add keypoints to all keypoints
			allKeypoints.addAll(kpList);

		}

		long end = System.currentTimeMillis();
		Utils.pl("Feature time: " + (end - start) + "ms");
		Utils.pl("allKeypoints.size(): " + allKeypoints.size());

		return allKeypoints;

	}

	public List<KeyPoint> filterFeatures(List<KeyPoint> features) {

		// runtime parameters
		int cellSize = Parameters.<Integer>get("cellSize");
		int binMinCapacity = Parameters.<Integer>get("binMinCapacity");
		int binMaxCapacity = Parameters.<Integer>get("binMaxCapacity");
		int maxFeatures = Parameters.<Integer>get("maxFeatures");

		// created sorted list of features (descending)
		int MAX_RESPONSE_VALUE = 256;
		HashMap<Integer, List<KeyPoint>> sortingTable = new HashMap<Integer, List<KeyPoint>>();

		// // group features by response value
		for (int i = 0; i <= MAX_RESPONSE_VALUE; i++) {
			sortingTable.put(i, new ArrayList<KeyPoint>());
		}

		for (KeyPoint kp : features) {
			sortingTable.get((int) kp.response).add(kp);
		}

		List<KeyPoint> sortedFeatures = new ArrayList<KeyPoint>();

		for (int i = MAX_RESPONSE_VALUE; i >= 0; i--) {
			sortedFeatures.addAll(sortingTable.get(i));
		}

		// initialize hash table for features
		HashMap<String, List<KeyPoint>> binnedFeatures = new HashMap<String, List<KeyPoint>>();

		int numRows = (int) Math.ceil(Parameters.<Integer>get("height").doubleValue() / cellSize);
		int numCols = (int) Math.ceil(Parameters.<Integer>get("width").doubleValue() / cellSize);

		for (int row = 0; row < numRows; row++) {
			for (int col = 0; col < numCols; col++) {
				// features are hashed with keys representing the row and column of the bin they
				// belong to. for example, a feature in the top left bin will have key "0,0" and
				// a feature in the bin at row 4 column 6 will have the key "4,6"
				binnedFeatures.put(row + "," + col, new ArrayList<KeyPoint>());
			}
		}

		// load sorted features into hash table, hashed by 2D bin
		for (int i = 0; i < sortedFeatures.size(); i++) {
			KeyPoint kp = sortedFeatures.get(i);
			int row = (int) (kp.pt.y / cellSize);
			int col = (int) (kp.pt.x / cellSize);
			binnedFeatures.get(row + "," + col).add(kp);
		}

		// filter through the keypoints by bin
		int totalFeatures = 0;
		for (int row = 0; row < numRows; row++) {
			for (int col = 0; col < numCols; col++) {

				List<KeyPoint> bin = binnedFeatures.get(row + "," + col);

				// non-max suppression?
//				this.preSortedNMS(bin);

				// retain the best features
				if (bin.size() < binMinCapacity) {
					totalFeatures += bin.size();
					continue;
				} else {

					int numGoodKeypoints = (int) bin.stream().filter(kp -> kp.response >= NORMAL_FAST_THRESHOLD)
							.count();

					if (numGoodKeypoints >= binMaxCapacity) {
						binnedFeatures.put(row + "," + col, bin.subList(0, binMaxCapacity));
						totalFeatures += binMaxCapacity;
					} else if (numGoodKeypoints >= binMinCapacity) {
						binnedFeatures.put(row + "," + col, bin.subList(0, numGoodKeypoints));
						totalFeatures += numGoodKeypoints;
					} else {
						binnedFeatures.put(row + "," + col, bin.subList(0, binMinCapacity));
						totalFeatures += binMinCapacity;
					}
				}

			}
		}

		// if too many features, start pruning the largest bins
		if (totalFeatures > maxFeatures) {
			// create table where bins are hashed by their size
			HashMap<Integer, List<List<KeyPoint>>> binsBySize = new HashMap<Integer, List<List<KeyPoint>>>();

			for (String key : binnedFeatures.keySet()) {
				List<KeyPoint> bin = binnedFeatures.get(key);
				if (binsBySize.get(bin.size()) == null) {
					binsBySize.put(bin.size(), new ArrayList<List<KeyPoint>>());
				}
				binsBySize.get(bin.size()).add(bin);
			}

			// get largest bin size
			int maxSize = Collections.max(binsBySize.keySet());

			// for however many iterations needed, remove a weak feature from the largest
			// bin
			for (int i = 0; i < totalFeatures - maxFeatures; i++) {
				List<List<KeyPoint>> bins = binsBySize.get(maxSize);

				// if we have removed all bins of this size, go down a size
				if (bins.size() == 0) {
					bins = binsBySize.get(--maxSize);
				}

				// otherwise, remove a keypoint from the first bin in the list and move the bin
				// down a size
				List<KeyPoint> bin = bins.get(0);
				bin.remove(bin.size() - 1);
				bins.remove(bin);
				if (binsBySize.get(bin.size()) == null) {
					binsBySize.put(bin.size(), new ArrayList<List<KeyPoint>>());
				}
				binsBySize.get(bin.size()).add(bin);

			}
		}

		// rebuild the list
		List<KeyPoint> filteredFeatures = new ArrayList<KeyPoint>();

		for (int row = 0; row < numRows; row++) {
			for (int col = 0; col < numCols; col++) {
				filteredFeatures.addAll(binnedFeatures.get(row + "," + col));
			}
		}

		return filteredFeatures;

	}

	public void setAngles(List<KeyPoint> filteredFeatures, List<Mat> pyramid, int patchSize) {

		// create lists from pyramid
		List<byte[]> buffers = new ArrayList<byte[]>();
		List<Integer> widths = new ArrayList<Integer>();
		List<Integer> heights = new ArrayList<Integer>();
		List<Float> scaleFactors = new ArrayList<Float>();

		float currentSF = 1;
		for (int i = 0; i < pyramid.size(); i++) {

			Mat borderedImage = new Mat();

			// add border to pyramid
			Core.copyMakeBorder(pyramid.get(i), borderedImage, patchSize, patchSize, patchSize, patchSize,
					Core.BORDER_REFLECT_101 + Core.BORDER_ISOLATED);

			widths.add(borderedImage.cols());
			heights.add(borderedImage.rows());

			scaleFactors.add(currentSF);
			currentSF *= this.SCALE_FACTOR;

			byte[] buffer = new byte[widths.get(i) * heights.get(i)];

			borderedImage.get(0, 0, buffer);
			buffers.add(buffer);
		}

		List<Integer> u_max = this.getUMax(patchSize);

		this.ICAngles(buffers, widths, heights, scaleFactors, filteredFeatures, u_max, patchSize);

	}

	// pass in the image pyramid buffers (with reflected borders), the widths and
	// heights of each octave, the scale of each octave [1, 1.2, 1.44, ...], the
	// keypoints, and the u_max list
	//
	// result is that the angles of all the keypoints are calculated
	public void ICAngles(List<byte[]> imgBuffers, List<Integer> widths, List<Integer> heights, List<Float> scaleFactors,
			List<KeyPoint> pts, List<Integer> u_max, int patchSize) {

		for (int ptidx = 0; ptidx < pts.size(); ptidx++) {

			int octave = pts.get(ptidx).octave;

			byte[] imgBuffer = imgBuffers.get(octave);
			int width = widths.get(octave);
			int height = heights.get(octave);
			float sf = scaleFactors.get(octave);

			int half_k = (int) (patchSize / 2);

			int centerX = (int) (pts.get(ptidx).pt.x / sf) + patchSize;
			int centerY = (int) (pts.get(ptidx).pt.y / sf) + patchSize;

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

					int val_plus = Byte.toUnsignedInt(imgBuffer[rowMajor(x, y, width)]);
					y = centerY - v;
					int val_minus = Byte.toUnsignedInt(imgBuffer[rowMajor(x, y, width)]);

					v_sum += (val_plus - val_minus);
					m_10 += u * (val_plus + val_minus);
				}
				m_01 += v * v_sum;
			}

			pts.get(ptidx).angle = (float) Core.fastAtan2((float) m_01, (float) m_10);
		}
	}

	public List<Integer> getUMax(int patchSize) {

		int halfPatchSize = patchSize / 2;
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

		return umax;

	}

	public static int rowMajor(int x, int y, int width) {
		return width * y + x;
	}

}
