package lumoslam;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;

import Jama.Matrix;
import arucomapping.ArUcoMap;
import initializers.Initializer;
import initializers.NNAcceleratedInitializer;
import placerecognition.BoWVector;
import runtimevars.Parameters;
import toolbox.Utils;
import types.ImageData;
import types.Point3D;
import types.Pose;

public class Map {

	protected boolean initialized = false;

	protected long latestKFID = 0;
	protected List<Keyframe> keyframes = new ArrayList<Keyframe>();
	protected Keyframe currentKeyframe = null;
	protected Initializer initializer = new NNAcceleratedInitializer(this);

	protected List<Point3D> allPoints = new ArrayList<Point3D>();
	protected List<MapPoint> allMapPoints = new ArrayList<MapPoint>();

	protected List<MovingModel> movingObjects = new ArrayList<MovingModel>();

	// groupings of triangulated map points by their coordinates, to be used for
	// faster global tracking.
	// the map point at (2, 0.4, 2.7) may have the key "0,0,0" if the CLUSTER_LENGTH
	// is 3 ((int)2 / 3, (int)0.4 / 3, (int)2.7 / 3)
	protected HashMap<String, List<MapPoint>> mapPointsByPartition = new HashMap<String, List<MapPoint>>();

	// map points that have been triangulated, but not checked for pruning
	protected List<MapPoint> uncheckedMapPoints = new ArrayList<MapPoint>();

	// map points that have had observations added since the calculation of their
	// principle descriptor (HashMap to avoid duplicates)
	protected HashMap<MapPoint, Boolean> mapPointsNeedingUpdatedPD = new HashMap<MapPoint, Boolean>();

	// marker-based map for demos that utilize markers in place of real-time user
	// input and for ground truth comparisons
	protected ArUcoMap markerMap = null;

	protected List<Keyframe> loopDetectedKeyframes = new ArrayList<Keyframe>();

	// synchronization lock for loop closure
	public Object loopClosureLock = new Object();

	// a loop closure is in progress
	protected boolean loopBeingClosed = false;

	// frame number of the last time a loop was closed
	protected long lastLoopClosed = -1;

	// frame number of the current keyframe when the last loop was closed (may be
	// different from lastLoopClosed if loop closure takes a long time)
	protected long lastLoopClosedTime = -1;

	public Map() {

		this.init();

	}

	public void init() {

		if (Parameters.<Boolean>get("usingMarkers")) {
			this.generateMarkerMap(Parameters.<String>get("markerMapFile"),
					Parameters.<List<List<Integer>>>get("movingObjectMarkers"));
		}

	}

	public void generateMarkerMap(String mapFile, List<List<Integer>> movingMarkerIds) {
		this.markerMap = new ArUcoMap(mapFile, movingMarkerIds);
	}

	public void createMapPointTableAndList(List<KeyPoint> keypoints, List<MapPoint> preExistingMapPoints,
			List<MapPoint> outMapPoints, HashMap<MapPoint, List<Integer>> outMapPointIndices) {

		outMapPoints.clear();
		outMapPointIndices.clear();

		// initialize map point list with null values
		for (int i = 0; i < keypoints.size(); i++) {
			outMapPoints.add(null);
		}

		// create table of keypoint indices based on location
		HashMap<String, List<Integer>> keypointIndexTable = new HashMap<String, List<Integer>>();
		for (int i = 0; i < keypoints.size(); i++) {
			Point pt = keypoints.get(i).pt;
			String key = Math.round(pt.x) + "," + Math.round(pt.y);
			if (keypointIndexTable.get(key) == null) {
				keypointIndexTable.put(key, new ArrayList<Integer>());
			}
			keypointIndexTable.get(key).add(i);
		}

		// load map points list and table
		for (String key : keypointIndexTable.keySet()) {
			List<Integer> indices = keypointIndexTable.get(key);

			// ideally, keypoints that are close together would map to the same MapPoint,
			// but this might not be reflected in preExistingMapPoints. to handle this,
			// select a default map point from preExistingMapPoints (if there is one) and
			// use it as a fall back for these indices if any of them don't have an
			// associated map point from preExistingMapPoints. otherwise, use the map point
			// given in preExistingMapPoints for each specific keypoint

			// get a default map point
			MapPoint defaultMP = new MapPoint();
			boolean defaultUsed = false;
			boolean defaultFound = false;
			if (preExistingMapPoints != null) {
				for (int i = 0; i < indices.size() && !defaultFound; i++) {
					if (preExistingMapPoints != null && preExistingMapPoints.get(indices.get(i)) != null) {
						defaultMP = preExistingMapPoints.get(indices.get(i));
						defaultFound = true;
					}
				}
			}
			// for each index, add the corresponding map point (or default map point to the
			// list)
			for (Integer idx : indices) {

				MapPoint preExistingMP = null;
				if (preExistingMapPoints != null) {
					preExistingMP = preExistingMapPoints.get(idx);
				}
				MapPoint mapPointToUse = preExistingMP;

				if (preExistingMP == null) {
					mapPointToUse = defaultMP;
					defaultUsed = true;
				}

				outMapPoints.set(idx, mapPointToUse);

				// set indices
				if (outMapPointIndices.get(mapPointToUse) == null) {
					outMapPointIndices.put(mapPointToUse, new ArrayList<Integer>());
				}
				outMapPointIndices.get(mapPointToUse).add(idx);
			}

			// check if new map point needs registration
			if (!defaultFound && defaultUsed) {
				this.registerMapPoint(defaultMP);
			}
		}

	}

	public synchronized Keyframe registerInitialKeyframe(ImageData imageData, long frameNum, String frameTitle) {
		Keyframe keyframe = new Keyframe(this.latestKFID++);
		keyframe.setFrameNum(frameNum);
		keyframe.setFrameTitle(frameTitle);
		Pose pose = new Pose();
		pose.setFixed(true);
		keyframe.setPose(pose);
		keyframe.setKeypoints(imageData.getKeypoints());
		keyframe.setKeypointsList(imageData.getKeypoints().toList());
		keyframe.setDescriptors(Utils.mergeDescriptorList(imageData.getMostRecentDescriptors()));
		keyframe.setDescriptorsList(imageData.getMostRecentDescriptors());
		keyframe.setDescriptorsMatrixList(Utils.matsToMatricesByte(keyframe.getDescriptorsList()));
		Utils.pl("keyframe.getDescriptorsMatrixList(): " + keyframe.getDescriptorsMatrixList());
		keyframe.getMostRecentDescriptorsList().clear();
		keyframe.getMostRecentDescriptorsList().addAll(imageData.getMostRecentDescriptors());

		this.createMapPointTableAndList(keyframe.getKeypointsList(), null, keyframe.getMapPoints(),
				keyframe.getMapPointIndices());

		keyframe.registerObservations(this);

		// place recognition vector
		BoWVector vector = new BoWVector();
		Mat bestDescriptors = BoWVector.getBestDescriptors(keyframe.getKeypoints(), keyframe.getDescriptors(), 0.5);
		vector.compute(bestDescriptors);
		keyframe.setBowVector(vector);

		this.keyframes.add(keyframe);
		this.currentKeyframe = keyframe;

		return keyframe;
	}

	// given a pose, keypoints, descriptors, and a list of the map points that are
	// associated with the corresponding descriptors/keypoints, construct a new
	// keyframe and add it to the map. NOTE: preExistingMapPoints may contain null
	// values
	public synchronized Keyframe registerNewKeyframe(long frameNum, String frameTitle, Pose pose,
			MatOfKeyPoint keypoints, Mat descriptors, List<MapPoint> preExistingMapPoints) {

		Keyframe keyframe = new Keyframe(this.latestKFID++);
		keyframe.setFrameNum(frameNum);
		keyframe.setFrameTitle(frameTitle);
		keyframe.setPose(pose);
		keyframe.setKeypoints(keypoints);
		keyframe.setKeypointsList(keypoints.toList());
		keyframe.setDescriptors(descriptors);
		List<Mat> descriptorsList = Utils.descriptorList(descriptors);
		keyframe.setDescriptorsList(descriptorsList);
		keyframe.setDescriptorsMatrixList(Utils.matsToMatricesByte(keyframe.getDescriptorsList()));

		keyframe.getMostRecentDescriptorsList().clear();
		keyframe.getMostRecentDescriptorsList().addAll(descriptorsList);

		this.createMapPointTableAndList(keyframe.getKeypointsList(), preExistingMapPoints, keyframe.getMapPoints(),
				keyframe.getMapPointIndices());

		keyframe.registerObservations(this);

		keyframe.registerPreviousKeyframe(this.currentKeyframe);
		this.currentKeyframe.registerNextKeyframe(keyframe);

		// place recognition vector
		BoWVector vector = new BoWVector();
		Mat bestDescriptors = BoWVector.getBestDescriptors(keyframe.getKeypoints(), keyframe.getDescriptors(), 0.5);
		vector.compute(bestDescriptors);
		keyframe.setBowVector(vector);

		this.keyframes.add(keyframe);
		this.currentKeyframe = keyframe;

		return keyframe;

	}

	public void testPointTransform(Keyframe keyframe) {

		List<Point3D> points = keyframe.getMapPoints().stream().filter(mp -> mp.getPoint() != null)
				.map(mp -> mp.getPoint()).collect(Collectors.toList());

		Pose pose = keyframe.getPose();
		Matrix transform = pose.getHomogeneousMatrix();
		for (Point3D p : points) {
			Point3D pCopy = new Point3D(p);
			Point3D p0 = pose.transformPoint(pCopy);
			Matrix p1 = transform.times(pCopy.getHomogeneousMatrix());
			Utils.p("p0: " + p0.getX() + ",\t" + p0.getY() + ",\t" + p0.getZ() + "\t\t\t\t");
			Utils.pl("p1: " + p1.get(0, 0) + ",\t" + p1.get(1, 0) + ",\t" + p1.get(2, 0));
		}

	}

	public void registerMovingObject(MovingModel mo) {
		for (int i = 0; i < mo.getMapPoints().size(); i++) {
			MapPoint mp = mo.getMapPoints().get(i);
			mp.setAssociatedObject(mo);
			this.purgeMapPoint(mp);
		}

		// register user-provided label for moving object
		if (Parameters.<List<String>>get("movingObjectLabels").size() > this.movingObjects.size()) {
			mo.setLabel(Parameters.<List<String>>get("movingObjectLabels").get(this.movingObjects.size()));
		}

		this.movingObjects.add(mo);
	}

	public void purgeMapPoint(MapPoint mp) {
		this.uncheckedMapPoints.remove(mp);
		Point3D point = mp.getPoint();
		this.getAllMapPoints().remove(mp);
		this.unPartitionMapPoint(mp);
		if (point != null) {
			this.getAllPoints().remove(point);
		}
	}

	public void registerMovingObjects(List<MovingModel> mos) {
		for (MovingModel mo : mos) {
			this.registerMovingObject(mo);
		}
	}

	// given a moving object, move neighboring points out of the map and into the
	// moving object structure for more robust tracking
	public void soakUpPoints(MovingModel movingObject) {

		double INCLUSION_THRESHOLD = 1.3;

		HashMap<MapPoint, Boolean> mapPointsToAdd = new HashMap<MapPoint, Boolean>();

		for (MapPoint mp : movingObject.getMapPoints()) {

			Point3D point = mp.getPoint();

			if (point == null) {
				continue;
			}

			// get neighbors
			String key = this.getPartitionKey(point);
			String[] keySplit = key.split(",");
			int xCenter = Integer.parseInt(keySplit[0]);
			int yCenter = Integer.parseInt(keySplit[1]);
			int zCenter = Integer.parseInt(keySplit[2]);

			List<MapPoint> neighbors = new ArrayList<MapPoint>();
			int RADIUS = 1;
			for (int x = xCenter - RADIUS; x <= xCenter + RADIUS; x++) {
				for (int y = yCenter - RADIUS; y <= yCenter + RADIUS; y++) {
					for (int z = zCenter - RADIUS; z <= zCenter + RADIUS; z++) {
						String newKey = x + "," + y + "," + z;
						if (this.mapPointsByPartition.get(newKey) == null) {
							continue;
						}
						neighbors.addAll(this.mapPointsByPartition.get(newKey));
					}
				}
			}

			for (MapPoint neighbor : neighbors) {
				Point3D neighborPoint = neighbor.getPoint();
				if (neighborPoint == null) {
					continue;
				}
				double dist = Utils.pointDistance(point, neighborPoint);
				if (dist < INCLUSION_THRESHOLD) {
					mapPointsToAdd.put(neighbor, true);
				}
			}

		}

		// remove each point from the map and add to moving object
		for (MapPoint mpToAdd : mapPointsToAdd.keySet()) {
			mpToAdd.setAssociatedObject(movingObject);
			this.purgeMapPoint(mpToAdd);
			movingObject.getMapPoints().add(mpToAdd);
		}

	}

	public List<Pose> getCameras() {
		List<Pose> cameras = new ArrayList<Pose>();
		for (int i = 0; i < this.keyframes.size(); i++) {
			cameras.add(this.keyframes.get(i).getPose());
		}
		return cameras;
	}

	public String getPartitionKey(Point3D point) {

		double clusterLength = Parameters.<Double>get("clusterLength");

		if (point == null) {
			return "";
		}

		int x = (int) (point.getX() / clusterLength);
		int y = (int) (point.getY() / clusterLength);
		int z = (int) (point.getZ() / clusterLength);

		String key = x + "," + y + "," + z;

		return key;
	}

	public synchronized void partitionMapPoint(MapPoint mp) {

		Point3D point = mp.getPoint();

		if (point == null) {
			return;
		}

		String key = this.getPartitionKey(point);

		if (this.mapPointsByPartition.get(key) == null) {
			this.mapPointsByPartition.put(key, new ArrayList<MapPoint>());
		}

		this.mapPointsByPartition.get(key).add(mp);

	}

	public synchronized void partitionMapPoints(List<MapPoint> mps) {

		for (int i = 0; i < mps.size(); i++) {
			this.partitionMapPoint(mps.get(i));
		}

	}

	public synchronized void unPartitionMapPoint(MapPoint mp) {

		if (mp.getPoint() == null) {
			return;
		}
		String key = this.getPartitionKey(mp.getPoint());
		List<MapPoint> cluster = this.mapPointsByPartition.get(key);
		if (cluster == null) {
			return;
		}

		boolean containedMapPoint = cluster.remove(mp);

	}

	public void registerMapPoint(MapPoint mp) {
		this.allMapPoints.add(mp);
	}

	public void registerPoint(Point3D point) {
		this.allPoints.add(point);
	}

	// copy information from secondayMapPoint into primaryMapPoint and update
	// references of secondaryMapPoint to point to primaryMapPoint
	public synchronized void mergeMapPoint(MapPoint primaryMapPoint, MapPoint secondaryMapPoint) {

		// remove secondary map point from map
		this.getAllMapPoints().remove(secondaryMapPoint);
		this.getUncheckedMapPoints().remove(secondaryMapPoint);
		this.getMapPointsNeedingUpdatedPD().remove(secondaryMapPoint);
		this.unPartitionMapPoint(secondaryMapPoint); // this may not work

		// for each observation of secondary map point, update the map point list and
		// map point indices list in the corresponding keyframe
		for (Observation o : secondaryMapPoint.getObservations()) {

			List<Integer> indices = new ArrayList<Integer>();
			for (int i = 0; i < o.getKeyframe().getMapPoints().size(); i++) {
				if (o.getKeyframe().getMapPoints().get(i) == secondaryMapPoint
						|| o.getKeyframe().getMapPoints().get(i) == primaryMapPoint) {
					o.getKeyframe().getMapPoints().set(i, primaryMapPoint);
					indices.add(i);
				}
			}

			o.getKeyframe().getMapPointIndices().remove(secondaryMapPoint);
			o.getKeyframe().getMapPointIndices().put(primaryMapPoint, indices);

		}

		// copy observations from secondary map point to primary map point
		primaryMapPoint.getObservations().addAll(secondaryMapPoint.getObservations());

		// mark the map point as merged so other processes (like principal descriptor
		// updates) can recognize it
		secondaryMapPoint.setMergedTo(primaryMapPoint);

	}

	public List<Keyframe> getKeyframes() {
		return keyframes;
	}

	public void setKeyframes(List<Keyframe> keyframes) {
		this.keyframes = keyframes;
	}

	public Keyframe getCurrentKeyframe() {
		return currentKeyframe;
	}

	public void setCurrentKeyframe(Keyframe currentKeyframe) {
		this.currentKeyframe = currentKeyframe;
	}

	public Initializer getInitializer() {
		return initializer;
	}

	public void setInitializer(Initializer initializer) {
		this.initializer = initializer;
	}

	public boolean isInitialized() {
		return initialized;
	}

	public void setInitialized(boolean initialized) {
		this.initialized = initialized;
	}

	public List<Point3D> getAllPoints() {
		return allPoints;
	}

	public void setAllPoints(List<Point3D> allPoints) {
		this.allPoints = allPoints;
	}

	public List<MapPoint> getAllMapPoints() {
		return allMapPoints;
	}

	public void setAllMapPoints(List<MapPoint> allMapPoints) {
		this.allMapPoints = allMapPoints;
	}

	public HashMap<String, List<MapPoint>> getMapPointsByPartition() {
		return mapPointsByPartition;
	}

	public void setMapPointsByPartition(HashMap<String, List<MapPoint>> mapPointsByPartition) {
		this.mapPointsByPartition = mapPointsByPartition;
	}

	public List<MapPoint> getUncheckedMapPoints() {
		return uncheckedMapPoints;
	}

	public void setUncheckedMapPoints(List<MapPoint> uncheckedMapPoints) {
		this.uncheckedMapPoints = uncheckedMapPoints;
	}

	public long getLatestKFID() {
		return latestKFID;
	}

	public void setLatestKFID(long latestKFID) {
		this.latestKFID = latestKFID;
	}

	public HashMap<MapPoint, Boolean> getMapPointsNeedingUpdatedPD() {
		return mapPointsNeedingUpdatedPD;
	}

	public void setMapPointsNeedingUpdatedPD(HashMap<MapPoint, Boolean> mapPointsNeedingUpdatedPD) {
		this.mapPointsNeedingUpdatedPD = mapPointsNeedingUpdatedPD;
	}

	public List<MovingModel> getMovingObjects() {
		return movingObjects;
	}

	public void setMovingObjects(List<MovingModel> movingObjects) {
		this.movingObjects = movingObjects;
	}

	public ArUcoMap getMarkerMap() {
		return markerMap;
	}

	public void setMarkerMap(ArUcoMap markerMap) {
		this.markerMap = markerMap;
	}

	public List<Keyframe> getLoopDetectedKeyframes() {
		return loopDetectedKeyframes;
	}

	public void setLoopDetectedKeyframes(List<Keyframe> loopDetectedKeyframes) {
		this.loopDetectedKeyframes = loopDetectedKeyframes;
	}

	public boolean isLoopBeingClosed() {
		return loopBeingClosed;
	}

	public void setLoopBeingClosed(boolean loopBeingClosed) {
		this.loopBeingClosed = loopBeingClosed;
	}

	public long getLastLoopClosed() {
		return lastLoopClosed;
	}

	public void setLastLoopClosed(long lastLoopClosed) {
		this.lastLoopClosed = lastLoopClosed;
	}

	public long getLastLoopClosedTime() {
		return lastLoopClosedTime;
	}

	public void setLastLoopClosedTime(long lastLoopClosedTime) {
		this.lastLoopClosedTime = lastLoopClosedTime;
	}

}
