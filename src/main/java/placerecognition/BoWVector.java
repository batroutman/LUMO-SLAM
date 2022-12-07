package placerecognition;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;

import toolbox.Utils;

public class BoWVector implements PlaceRecognitionVector<BoWVector> {

	public Vocabulary vocabulary = Vocabulary.global;

	// compressed vector for the image. key is the ID of the descriptor (from
	// BoWIndices) and value is the number of occurrences in the image divided by
	// number of descriptors in vector
	protected HashMap<Integer, Double> vector = new HashMap<Integer, Double>();

	@Override
	public void compute(Mat descriptors) {

		this.vector.clear();
		List<List<Byte>> descriptorByteLists = Utils.getDescriptorByteLists(descriptors);

		// for each descriptor, find best index in vocab tree
		for (List<Byte> descriptor : descriptorByteLists) {
			int bestIndex = this.getIndex(descriptor, this.vocabulary.root, 2);
			Double val = this.vector.get(bestIndex);
			if (val == null) {
				this.vector.put(bestIndex, 1.0);
			} else {
				this.vector.put(bestIndex, val + 1);
			}
		}

		// normalize vector
		for (Integer key : this.vector.keySet()) {
			this.vector.put(key, this.vector.get(key) / descriptorByteLists.size());
		}

	}

	@Override
	public double getScore(BoWVector prv) {

		// collect all keys (removing duplicates)
		HashMap<Integer, Boolean> allKeys = new HashMap<Integer, Boolean>();
		for (Integer id : this.vector.keySet()) {
			allKeys.put(id, true);
		}
		for (Integer id : prv.vector.keySet()) {
			allKeys.put(id, true);
		}

		// compute L1 norm value for each occurrence
		double L1 = 0;
		for (Integer id : allKeys.keySet()) {
			Double v0 = this.vector.get(id);
			Double v1 = prv.vector.get(id);

			v0 = v0 == null ? 0 : v0;
			v1 = v1 == null ? 0 : v1;

			L1 += Math.abs(v0 - v1);
		}

		return 1 - L1 * 0.5;

	}

	public static int hammingDistance(List<Byte> d0, List<Byte> d1) {

		int total = 0;
		for (int i = 0; i < d0.size(); i++) {
			int xor = d0.get(i) ^ d1.get(i);
			total += Integer.bitCount(xor & 0xFF);
		}
		return total;

	}

	protected int getIndex(List<Byte> descriptor, Vocabulary.Node root, int level) {

//		Utils.pl("root: " + root);

		if (level <= 0 && (root.childrenIds == null || root.childrenIds.size() == 0)) {
			return root.id;
		}

		// record lowest hamming distance child
		int bestId = root.isLeaf ? root.id : -1;
		int bestHamming = root.isLeaf ? hammingDistance(descriptor, root.descriptor) : 256;
		for (int i = 0; i < root.childrenIds.size(); i++) {
			int hamming = hammingDistance(descriptor, this.vocabulary.nodes.get(root.childrenIds.get(i)).descriptor);
			if (hamming < bestHamming) {
				bestHamming = hamming;
				bestId = root.childrenIds.get(i);
			}
		}

		if (bestId == root.id || root.childrenIds.size() == 0) {
			return root.id;
		}

		return this.getIndex(descriptor, this.vocabulary.nodes.get(bestId), level - 1);

	}

	public static Mat getBestDescriptors(MatOfKeyPoint keypoints, Mat descriptors, double prop) {

		List<KeyPoint> keypointsList = keypoints.toList();
		List<List<Byte>> descriptorsList = Utils.getDescriptorByteLists(descriptors);

		class KPD {
			public KeyPoint keypoint;
			public List<Byte> descriptor;
		}

		// sorting table based on FAST response
		HashMap<Integer, List<KPD>> kpds = new HashMap<Integer, List<KPD>>();
		for (int i = 0; i < 256; i++) {
			kpds.put(i, new ArrayList<KPD>());
		}
		int numFiltered = 0;
		for (int i = 0; i < keypointsList.size(); i++) {
			KPD kpd = new KPD();
			kpd.keypoint = keypointsList.get(i);
			kpd.descriptor = descriptorsList.get(i);
			kpds.get((int) kpd.keypoint.response).add(kpd);
			numFiltered++;
		}

		// collect the highest scoring descriptors
		int numHighest = (int) (prop * numFiltered);
		List<KPD> finalKPDs = new ArrayList<KPD>();
		for (int i = 255; i >= 0 && finalKPDs.size() < numHighest; i--) {
			List<KPD> scoredKPDs = kpds.get(i);
			if (numHighest - finalKPDs.size() >= scoredKPDs.size()) {
				finalKPDs.addAll(scoredKPDs);
			} else {
				finalKPDs.addAll(scoredKPDs.subList(0, numHighest - finalKPDs.size()));
			}
		}

		List<List<Byte>> finalDescriptors = finalKPDs.stream().map(kpd -> kpd.descriptor).collect(Collectors.toList());
		Mat finalDescriptorsMat = Utils.byteListsToMat(finalDescriptors);

		return finalDescriptorsMat;

	}

}
