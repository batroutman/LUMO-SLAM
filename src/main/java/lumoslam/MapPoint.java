package lumoslam;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import Jama.Matrix;
import toolbox.Utils;
import types.Point3D;

public class MapPoint {

	protected Point3D point = null;
	protected List<Observation> observations = new ArrayList<Observation>();
	protected Observation principleObservation = null;
	protected Integer principleDescriptorIndex = null;
	protected Long frameTriangulated = null;
	protected long timesTracked = 0; // times tracked by point inclusion tracking
	protected long timesLost = 0; // times point was supposed to be tracked but wasn't
	protected long timesRegistered = 0; // times tracked by keyframe tracking
	protected MovingModel associatedObject = null;
	protected MapPoint mergedTo = null;

	public MapPoint() {

	}

	// look through the list of observations and set the principle observation to be
	// the one with the lowest median distance from the other observations
	public void computePrincipleObservation() {

		List<Observation> observationsCopy = new ArrayList(this.observations);

		// get all descriptors from observations
		HashMap<Matrix, Observation> descriptorsToObservations = new HashMap<Matrix, Observation>();
		HashMap<Matrix, Integer> descriptorsToIndex = new HashMap<Matrix, Integer>();
		for (Observation obsv : observationsCopy) {
			List<Integer> indices = obsv.getKeyframe().getMapPointIndices().get(this);
			for (Integer idx : indices) {
				descriptorsToObservations.put(obsv.getKeyframe().getDescriptorsMatrixList().get(idx), obsv);
				descriptorsToIndex.put(obsv.getKeyframe().getDescriptorsMatrixList().get(idx), idx);
			}
		}
		List<Matrix> descriptors = new ArrayList<Matrix>(descriptorsToObservations.keySet());

		if (descriptors.size() == 0) {
			this.principleObservation = null;
			this.principleDescriptorIndex = 0;
			return;
		}

		// compute distance matrix
		List<List<Integer>> distances = new ArrayList<List<Integer>>();
		for (int i = 0; i < descriptors.size(); i++) {
			List<Integer> dists = new ArrayList<Integer>();
			for (int j = 0; j < descriptors.size(); j++) {
				dists.add(0);
			}
			distances.add(dists);
		}
		for (int i = 0; i < descriptors.size() - 1; i++) {
			for (int j = 1; j < descriptors.size(); j++) {
				double distance = Utils.descriptorDistance(descriptors.get(i), descriptors.get(j));
				distances.get(i).set(j, (int) distance);
				distances.get(j).set(i, (int) distance);
			}
		}

		// get median distance for each descriptor
		for (List<Integer> dists : distances) {
			Collections.sort(dists);
		}

		int bestMedian = Integer.MAX_VALUE;
		int bestIdx = 0;
		for (int i = 0; i < distances.size(); i++) {
			List<Integer> dists = distances.get(i);
			int median = dists.get((int) 0.5 * (dists.size() - 1));
			if (median < bestMedian) {
				bestMedian = median;
				bestIdx = i;
			}
		}

		// set principle observation
		this.principleObservation = descriptorsToObservations.get(descriptors.get(bestIdx));
		this.principleDescriptorIndex = descriptorsToIndex.get(descriptors.get(bestIdx));

	}

	public Point3D getPoint() {
		return point;
	}

	public void setPoint(Point3D point, Long frameNum) {
		this.point = point;
		this.frameTriangulated = frameNum;
	}

	public List<Observation> getObservations() {
		return observations;
	}

	public void setObservations(List<Observation> observations) {
		this.observations = observations;
	}

	public Long getFrameTriangulated() {
		return frameTriangulated;
	}

	public void setFrameTriangulated(Long frameTriangulated) {
		this.frameTriangulated = frameTriangulated;
	}

	public long getTimesTracked() {
		return timesTracked;
	}

	public void setTimesTracked(long timesTracked) {
		this.timesTracked = timesTracked;
	}

	public void incTimesTracked() {
		this.timesTracked++;
	}

	public long getTimesRegistered() {
		return timesRegistered;
	}

	public void setTimesRegistered(long timesRegistered) {
		this.timesRegistered = timesRegistered;
	}

	public long getTimesLost() {
		return timesLost;
	}

	public void setTimesLost(long timesLost) {
		this.timesLost = timesLost;
	}

	public void incTimesLost() {
		this.timesLost++;
	}

	public Observation getPrincipleObservation() {
		return principleObservation;
	}

	public void setPrincipleObservation(Observation principleObservation) {
		this.principleObservation = principleObservation;
	}

	public Integer getPrincipleDescriptorIndex() {
		return principleDescriptorIndex;
	}

	public void setPrincipleDescriptorIndex(Integer principleDescriptorIndex) {
		this.principleDescriptorIndex = principleDescriptorIndex;
	}

	public MovingModel getAssociatedObject() {
		return associatedObject;
	}

	public void setAssociatedObject(MovingModel associatedObject) {
		this.associatedObject = associatedObject;
	}

	public MapPoint getMergedTo() {
		return mergedTo;
	}

	public void setMergedTo(MapPoint mergedTo) {
		this.mergedTo = mergedTo;
	}

}
