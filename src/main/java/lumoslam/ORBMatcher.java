package lumoslam;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

import org.opencv.core.Core;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.features2d.BFMatcher;
import org.opencv.features2d.DescriptorMatcher;

import lumoslam.Tracker.DescriptorInfo;
import toolbox.Utils;
import types.Correspondence2D2D;
import types.Dbl;

public class ORBMatcher {

	static DescriptorMatcher matcher = BFMatcher.create(Core.NORM_HAMMING, true);
	static DescriptorMatcher knnMatcher = BFMatcher.create(Core.NORM_HAMMING, false);

//	static int MATCH_THRESHOLD = 0; // mock
	static int MATCH_THRESHOLD = 40; // real 15

	public ORBMatcher() {
		this.init();
		// prevent linter from removing Utils because I use it on and off
		Utils.p("");
	}

	protected void init() {

	}

	public static List<Correspondence2D2D> matchDescriptors(List<KeyPoint> referenceKeypoints, Mat referenceDescriptors,
			List<KeyPoint> currentKeypoints, Mat currentDescriptors) {

		List<Correspondence2D2D> correspondences = new ArrayList<Correspondence2D2D>();

		DescriptorMatcher matcher = BFMatcher.create(Core.NORM_HAMMING, true);
		MatOfDMatch matches = new MatOfDMatch();

		// tries to find a match for each query (currentDescriptor) against the already
		// existing train (referenceDescriptor) set
		matcher.match(currentDescriptors, referenceDescriptors, matches);

		List<DMatch> listMatches = matches.toList();
		for (int i = 0; i < listMatches.size(); i++) {
			DMatch dmatch = listMatches.get(i);

			if (dmatch.distance <= MATCH_THRESHOLD) {
				Correspondence2D2D c = new Correspondence2D2D();
				c.setX0(referenceKeypoints.get(dmatch.trainIdx).pt.x);
				c.setY0(referenceKeypoints.get(dmatch.trainIdx).pt.y);
				c.setX1(currentKeypoints.get(dmatch.queryIdx).pt.x);
				c.setY1(currentKeypoints.get(dmatch.queryIdx).pt.y);
				c.computeLength();
				correspondences.add(c);
			}
		}

		blindPrune(correspondences);

		return correspondences;

	}

	public static List<Correspondence2D2D> matchDescriptors(List<KeyPoint> referenceKeypoints,
			List<Mat> referenceDescriptors, List<KeyPoint> currentKeypoints, Mat currentDescriptors, int threshold) {
		return matchDescriptors(referenceKeypoints, referenceDescriptors, currentKeypoints, currentDescriptors,
				threshold, true);
	}

	// match using the most recent descriptors and update them
	public static List<Correspondence2D2D> matchDescriptors(List<KeyPoint> referenceKeypoints,
			List<Mat> referenceDescriptors, List<KeyPoint> currentKeypoints, Mat currentDescriptors, int threshold,
			boolean blindPrune) {

		List<Correspondence2D2D> correspondences = new ArrayList<Correspondence2D2D>();

		DescriptorMatcher matcher = BFMatcher.create(Core.NORM_HAMMING, true);
		MatOfDMatch matches = new MatOfDMatch();

		// tries to find a match for each query (currentDescriptor) against the already
		// existing train (referenceDescriptor) set
		matcher.match(currentDescriptors, Utils.mergeDescriptorList(referenceDescriptors), matches);

		List<Mat> currentDescriptorList = Utils.descriptorList(currentDescriptors);

		List<DMatch> listMatches = matches.toList();
		for (int i = 0; i < listMatches.size(); i++) {
			DMatch dmatch = listMatches.get(i);

			if (dmatch.distance <= threshold) {
				Correspondence2D2D c = new Correspondence2D2D();
				c.setX0(referenceKeypoints.get(dmatch.trainIdx).pt.x);
				c.setY0(referenceKeypoints.get(dmatch.trainIdx).pt.y);
				c.setX1(currentKeypoints.get(dmatch.queryIdx).pt.x);
				c.setY1(currentKeypoints.get(dmatch.queryIdx).pt.y);
				c.computeLength();
				correspondences.add(c);

				// update most recent descriptor
				referenceDescriptors.set(dmatch.trainIdx, currentDescriptorList.get(dmatch.queryIdx));
			}
		}

		if (blindPrune) {
			blindPrune(correspondences);
		}

		return correspondences;

	}

	public static List<DMatch> matchDescriptors(Keyframe referenceFrame, Mat currentDescriptors) {
		return matchDescriptors(Utils.mergeDescriptorList(referenceFrame.getMostRecentDescriptorsList()),
				currentDescriptors);
	}

	public static List<DMatch> matchDescriptors(Mat referenceDescriptors, Mat currentDescriptors) {
		return matchDescriptors(referenceDescriptors, currentDescriptors, MATCH_THRESHOLD);
	}

	public static List<DMatch> matchDescriptors(Mat referenceDescriptors, Mat currentDescriptors, int threshold) {

		MatOfDMatch matches = new MatOfDMatch();

		// tries to find a match for each query (currentDescriptor) against the already
		// existing train (referenceDescriptor) set
		matcher.match(currentDescriptors, referenceDescriptors, matches);

		List<DMatch> matchList = matches.toList();
		List<DMatch> filteredMatches = new ArrayList<DMatch>();
		for (int i = 0; i < matchList.size(); i++) {
			if (matchList.get(i).distance <= threshold) {
				filteredMatches.add(matchList.get(i));
			}
		}

		return filteredMatches;

	}

	public static List<DMatch> guidedMatching(List<DescriptorInfo> referenceDescriptorInfo, List<Mat> descriptorsList,
			List<KeyPoint> keypointsList, double matchThreshold) {

		double CELL_LENGTH = 10;

		// load descriptors into hash table based on their keypoint location
		HashMap<String, List<Mat>> descriptorTable = new HashMap<String, List<Mat>>();
		HashMap<Mat, Integer> descriptorIndices = new HashMap<Mat, Integer>();
		for (int i = 0; i < descriptorsList.size(); i++) {
			descriptorIndices.put(descriptorsList.get(i), i);
			int x = (int) (keypointsList.get(i).pt.x / CELL_LENGTH);
			int y = (int) (keypointsList.get(i).pt.y / CELL_LENGTH);
			if (descriptorTable.get(x + "," + y) == null) {
				descriptorTable.put(x + "," + y, new ArrayList<Mat>());
			}
			descriptorTable.get(x + "," + y).add(descriptorsList.get(i));
		}

		// load reference descriptors into hash table based on their keypoint location
		HashMap<String, List<Mat>> referenceDescriptorTable = new HashMap<String, List<Mat>>();
		HashMap<Mat, Integer> referenceDescriptorIndices = new HashMap<Mat, Integer>();
		for (int i = 0; i < referenceDescriptorInfo.size(); i++) {
			referenceDescriptorIndices.put(referenceDescriptorInfo.get(i).descriptor, i);
			int x = (int) (referenceDescriptorInfo.get(i).projectionInfo.projection.x / CELL_LENGTH);
			int y = (int) (referenceDescriptorInfo.get(i).projectionInfo.projection.y / CELL_LENGTH);
			if (referenceDescriptorTable.get(x + "," + y) == null) {
				referenceDescriptorTable.put(x + "," + y, new ArrayList<Mat>());
			}
			referenceDescriptorTable.get(x + "," + y).add(referenceDescriptorInfo.get(i).descriptor);
		}

		// go through each set of reference descriptors and match against the
		// corresponding and neighboring partitions
		List<DMatch> allMatches = new ArrayList<DMatch>();
		int loweRejects = 0;
		for (String key : referenceDescriptorTable.keySet()) {
			List<Mat> referenceDescriptorPartition = referenceDescriptorTable.get(key);
			String[] splits = key.split(",");
			int x = Integer.parseInt(splits[0]);
			int y = Integer.parseInt(splits[1]);

			// gather neighboring partitions
			List<Mat> descriptorPartition = new ArrayList<Mat>();
			for (int i = -1; i <= 1; i++) {
				for (int j = -1; j <= 1; j++) {
					List<Mat> partition = descriptorTable.get((x + i) + "," + (y + j));
					if (partition != null) {
						descriptorPartition.addAll(partition);
					}
				}
			}

			if (descriptorPartition.size() == 0) {
				continue;
			}

			// perform matching
			List<MatOfDMatch> matchesList = new ArrayList<MatOfDMatch>();
			Mat referenceDescriptorPartitionMat = Utils.mergeDescriptorList(referenceDescriptorPartition);
			Mat descriptorPartitionMat = Utils.mergeDescriptorList(descriptorPartition);
			knnMatcher.knnMatch(referenceDescriptorPartitionMat, descriptorPartitionMat, matchesList, 2);

			// re-format match list
			List<List<DMatch>> matches = new ArrayList<List<DMatch>>();
			for (int i = 0; i < matchesList.size(); i++) {
				matches.add(matchesList.get(i).toList());
			}

			// check threshold and convert to correct indices
			double LOWE_RATIO = 0.8;
			for (int i = 0; i < matches.size(); i++) {
				DMatch bestMatch = matches.get(i).get(0);
				if (matches.get(i).size() > 1) {
					DMatch secondBestMatch = matches.get(i).get(1);
					KeyPoint bestKP = keypointsList
							.get(descriptorIndices.get(descriptorPartition.get(bestMatch.trainIdx)));
					KeyPoint secondBestKP = keypointsList
							.get(descriptorIndices.get(descriptorPartition.get(secondBestMatch.trainIdx)));
					if (bestKP.octave == secondBestKP.octave
							&& bestMatch.distance > LOWE_RATIO * secondBestMatch.distance) {
						loweRejects++;
						continue;
					}
				}

				if (bestMatch.distance < matchThreshold) {
					int trainIdx = descriptorIndices.get(descriptorPartition.get(bestMatch.trainIdx));
					int queryIdx = referenceDescriptorIndices.get(referenceDescriptorPartition.get(bestMatch.queryIdx));
					bestMatch.trainIdx = trainIdx;
					bestMatch.queryIdx = queryIdx;
					allMatches.add(bestMatch);
				}
			}

		}
		Utils.pl("loweRejects: " + loweRejects);
		return allMatches;

	}

	// prune correspondences with lengths greater than some value
	public static void blindPrune(List<Correspondence2D2D> correspondences) {
		Dbl avg = new Dbl(0);
		Dbl stdDev = new Dbl(0);
		Dbl median = new Dbl(0);

		Utils.basicStats(correspondences.stream().map(c -> c.getLength()).collect(Collectors.toList()), avg, stdDev,
				median);

		for (int i = 0; i < correspondences.size(); i++) {
			if (correspondences.get(i).getLength() > median.getValue() + 2 * stdDev.getValue()) {
				correspondences.remove(i);
				i--;
			}
		}
	}

	// prune correspondences with lengths greater than some value
	public static void blindPrune(List<Correspondence2D2D> correspondences, List<DMatch> matches) {
		Dbl avg = new Dbl(0);
		Dbl stdDev = new Dbl(0);
		Dbl median = new Dbl(0);

		Utils.basicStats(correspondences.stream().map(c -> c.getLength()).collect(Collectors.toList()), avg, stdDev,
				median);

		if (median.getValue() == 0) {
			return;
		}

		for (int i = 0; i < correspondences.size(); i++) {
			if (correspondences.get(i).getLength() > median.getValue() * 5) {
				correspondences.remove(i);
				matches.remove(i);
				i--;
			}
		}
	}

}
