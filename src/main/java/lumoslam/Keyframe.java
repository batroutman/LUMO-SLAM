package lumoslam;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;

import Jama.Matrix;
import placerecognition.BoWVector;
import toolbox.Utils;
import types.Pose;

public class Keyframe {

	long id = 0;
	long frameNum = 0;
	String frameTitle = "";
	Pose pose = new Pose();

	MatOfKeyPoint keypoints = null;
	Mat descriptors = null;
	List<KeyPoint> keypointsList = new ArrayList<KeyPoint>();
	List<Mat> descriptorsList = new ArrayList<Mat>();
	List<Matrix> descriptorsMatrixList = new ArrayList<Matrix>();

	BoWVector bowVector = null;

	boolean checkedForLoopClosure = false;

	// continually updated descriptors
	List<Mat> mostRecentDescriptorsList = new ArrayList<Mat>();

	List<MapPoint> mapPoints = new ArrayList<MapPoint>();

	// map map points to their indices in mapPoint, keypoints, descriptors, etc.
	HashMap<MapPoint, List<Integer>> mapPointIndices = new HashMap<MapPoint, List<Integer>>();

	// keyframe graphing
	Keyframe previousKeyframe = null;
	Keyframe nextKeyframe = null;
	List<Keyframe> connectedKeyframes = new ArrayList<Keyframe>();

	public Keyframe(long id) {
		this.id = id;
	}

	// given a keyframe, go through all of its map points and add an observation for
	// the corresponding keypoint in the keyframe
	public void registerObservations(Map map) {
		List<KeyPoint> keypointList = this.getKeypoints().toList();
		for (int i = 0; i < this.getMapPoints().size(); i++) {
			MapPoint mp = this.getMapPoints().get(i);
			mp.getObservations().add(new Observation(this, keypointList.get(i).pt, mp));
			if (mp.getObservations().size() == 1) {
				mp.computePrincipleObservation();
			} else {
				map.getMapPointsNeedingUpdatedPD().put(mp, true);
			}
		}
	}

	// return true if mapPointIndices is consistent with the map points list
	public boolean validMapPointMapping() {

		boolean result = true;

		// check that all map points in mapPointIndices exist in mapPoints (list)
		for (MapPoint mp : this.mapPointIndices.keySet()) {
			List<Integer> indices = this.mapPointIndices.get(mp);
			if (indices == null) {
				Utils.pl("WARNING: null indices");
			} else {
				for (Integer idx : indices) {
					if (this.mapPoints.get(idx) != mp) {
						Utils.pl("MapPoint " + mp + " not found at index " + idx + " from indices " + indices
								+ "first indexOf MapPoint " + mp + " : " + this.mapPoints.indexOf(mp));
						result = false;
					}
				}
			}
		}

		// check that all map points from list are in mapPointIndices
		for (int i = 0; i < this.mapPoints.size(); i++) {
			List<Integer> indices = this.mapPointIndices.get(this.mapPoints.get(i));
			if (indices == null) {
				Utils.pl("Indices not found (null) for MapPoint " + this.mapPoints.get(i) + " at index " + i);
				return false;
			} else {
				if (!indices.contains(i)) {
					Utils.pl("Indices does not contain proper index (" + i + ") for MapPoint " + this.mapPoints.get(i)
							+ ". Indices: " + indices);
					result = false;
				}
			}
		}

		return result;

	}

	public Pose getPose() {
		return pose;
	}

	public void setPose(Pose pose) {
		this.pose = pose;
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

	public List<MapPoint> getMapPoints() {
		return mapPoints;
	}

	public void setMapPoints(List<MapPoint> mapPoints) {
		this.mapPoints = mapPoints;
	}

	public Keyframe getPreviousKeyframe() {
		return previousKeyframe;
	}

	public void setPreviousKeyframe(Keyframe previousKeyframe) {
		this.previousKeyframe = previousKeyframe;
	}

	public Keyframe getNextKeyframe() {
		return nextKeyframe;
	}

	public void setNextKeyframe(Keyframe nextKeyframe) {
		this.nextKeyframe = nextKeyframe;
	}

	public List<Keyframe> getConnectedKeyframes() {
		return connectedKeyframes;
	}

	public void setConnectedKeyframes(List<Keyframe> connectedKeyframes) {
		this.connectedKeyframes = connectedKeyframes;
	}

	public void registerPreviousKeyframe(Keyframe kf) {
		this.previousKeyframe = kf;
		this.connectedKeyframes.add(kf);
	}

	public void registerNextKeyframe(Keyframe kf) {
		this.nextKeyframe = kf;
		this.connectedKeyframes.add(kf);
	}

	public long getFrameNum() {
		return frameNum;
	}

	public void setFrameNum(long frameNum) {
		this.frameNum = frameNum;
	}

	public List<KeyPoint> getKeypointsList() {
		return keypointsList;
	}

	public void setKeypointsList(List<KeyPoint> keypointsList) {
		this.keypointsList = keypointsList;
	}

	public List<Mat> getDescriptorsList() {
		return descriptorsList;
	}

	public void setDescriptorsList(List<Mat> descriptorsList) {
		this.descriptorsList = descriptorsList;
	}

	public HashMap<MapPoint, List<Integer>> getMapPointIndices() {
		return mapPointIndices;
	}

	public void setMapPointIndices(HashMap<MapPoint, List<Integer>> mapPointIndices) {
		this.mapPointIndices = mapPointIndices;
	}

	public List<Mat> getMostRecentDescriptorsList() {
		return mostRecentDescriptorsList;
	}

	public void setMostRecentDescriptorsList(List<Mat> mostRecentDescriptorsList) {
		this.mostRecentDescriptorsList = mostRecentDescriptorsList;
	}

	public long getID() {
		return id;
	}

	public void setID(long id) {
		this.id = id;
	}

	public List<Matrix> getDescriptorsMatrixList() {
		return descriptorsMatrixList;
	}

	public void setDescriptorsMatrixList(List<Matrix> descriptorsMatrixList) {
		this.descriptorsMatrixList = descriptorsMatrixList;
	}

	public String getFrameTitle() {
		return frameTitle;
	}

	public void setFrameTitle(String frameTitle) {
		this.frameTitle = frameTitle;
	}

	public BoWVector getBowVector() {
		return bowVector;
	}

	public void setBowVector(BoWVector bowVector) {
		this.bowVector = bowVector;
	}

	public boolean isCheckedForLoopClosure() {
		return checkedForLoopClosure;
	}

	public void setCheckedForLoopClosure(boolean checkedForLoopClosure) {
		this.checkedForLoopClosure = checkedForLoopClosure;
	}

}
