package runtimevars;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import Jama.Matrix;
import toolbox.Utils;

public final class Parameters {

	public static Map<String, Object> parameters = new HashMap<String, Object>();

	public static void setDefaultParameters() {

		// -------- input data -------- //
		parameters.put("inputDataPath", "../datasets/rgbd_dataset_freiburg3_structure_texture_far/"); // String
		parameters.put("bufferType", "TUM"); // String ("TUM", "mp4")
		parameters.put("bowVocabFile", "../BoW/ORBvoc.txt"); // String

		// -------- trajectory save data -------- //
		parameters.put("trajectorySavePath", "trajectory.txt"); // String
		parameters.put("keyframeTrajectorySavePath", "kftrajectory.txt"); // String
		parameters.put("movingObjectTrajectoryPathJSON", "moTrajectories.json"); // String
		parameters.put("movingObjectTrajectoryPathTXT", "moTrajectories"); // String
		parameters.put("markerSimTransformPath", "markerSimTransform.json"); // String

		// -------- camera parameters -------- //
		parameters.put("fx", 535.4f); // Float
		parameters.put("fy", 539.2f); // Float
		parameters.put("cx", 320.1f); // Float
		parameters.put("cy", 247.6f); // Float
		parameters.put("s", 0f); // Float
		List<Double> distCoeffs = new ArrayList<Double>();
		parameters.put("distCoeffs", distCoeffs); // List<Double>

		// -------- resolution information -------- //
		parameters.put("width", 640); // Integer
		parameters.put("height", 480); // Integer
		parameters.put("screenWidth", 1280); // Integer
		parameters.put("screenHeight", 960); // Integer

		// -------- initialization and point triangulation -------- //
		parameters.put("smartInitialization", true);
		parameters.put("parallaxRequirement", 2.0); // Double
		parameters.put("initializationReprojectionError", 4.0); // Double

		// -------- feature extraction -------- //
		parameters.put("cellSize", 40); // Integer
		parameters.put("binMinCapacity", 4); // Integer
		parameters.put("binMaxCapacity", 8); // Integer
		parameters.put("maxFeatures", 900); // Integer

		// -------- point clustering -------- //
		parameters.put("clusterLength", 3.0); // Double

		// -------- PnP Tracking -------- //
		parameters.put("PnPRansacReprojectionThreshold", 7.0f); // Float

		// -------- MO Registration -------- //
		parameters.put("maxRegistrationAttempts", 3); // Integer
		List<String> moLabels = new ArrayList<String>();
		moLabels.add("Moving Object 0");
		moLabels.add("Moving Object 1");
		moLabels.add("Moving Object 2");
		moLabels.add("Moving Object 3");
		moLabels.add("Moving Object 4");
		moLabels.add("Moving Object 5");
		moLabels.add("Moving Object 6");
		moLabels.add("Moving Object 7");
		moLabels.add("Moving Object 8");
		moLabels.add("Moving Object 9");
		parameters.put("movingObjectLabels", moLabels); // List<String>

		// -------- cubes -------- //
		List<List<Float>> cubes = new ArrayList<List<Float>>();
		List<Float> cube0 = new ArrayList<Float>();
		cube0.add(0f);
		cube0.add(4f);
		cube0.add(12f);
		cube0.add(53f);
		cube0.add(10f);
		cube0.add(250f);
		cube0.add(2f);
		cubes.add(cube0);

		parameters.put("cubes", cubes); // List<List<Float>>

		// -------- ArUco markers -------- //
		parameters.put("usingMarkers", false); // Boolean
		parameters.put("markerMapFile", ""); // String

		// this is a list of the marker IDs that are on moving objects. for the purposes
		// of this system, the markers with these IDs will simply be omitted from the
		// marker map to prevent moving markers from breaking the tracking of the marker
		// map.
		parameters.put("movingObjectMarkers", new ArrayList<List<Integer>>()); // List<List<String>>

		// -------- GUI Default Options -------- //
		parameters.put("ARView.showMapPoints", false); // Boolean

		parameters.put("ARView.showBoundingBoxes", true); // Boolean
	}

	public static void printParams() {
		Utils.pl("Parameters: ");
		for (String key : parameters.keySet()) {
			Utils.pl(String.format("%-48s", key + ":") + parameters.get(key));
		}
	}

	public static <T> T get(String var) {
		return (T) parameters.get(var);
	}

	public static <T> void put(String name, T var) {
		parameters.put(name, var);
	}

	public static Matrix getK() {
		Matrix K = Matrix.identity(3, 3);
		K.set(0, 0, (Float) parameters.get("fx"));
		K.set(0, 1, (Float) parameters.get("s"));
		K.set(0, 2, (Float) parameters.get("cx"));
		K.set(1, 1, (Float) parameters.get("fy"));
		K.set(1, 2, (Float) parameters.get("cy"));
		return K;
	}

	public static Matrix getK4x4() {
		Matrix K = Matrix.identity(4, 4);
		K.set(0, 0, (Float) parameters.get("fx"));
		K.set(0, 1, (Float) parameters.get("s"));
		K.set(0, 2, (Float) parameters.get("cx"));
		K.set(1, 1, (Float) parameters.get("fy"));
		K.set(1, 2, (Float) parameters.get("cy"));
		return K;
	}

	public static Mat getKMat() {
		Mat K = new Mat(3, 3, CvType.CV_64FC1);
		K.put(0, 0, (Float) parameters.get("fx"), (Float) parameters.get("s"), (Float) parameters.get("cx"), 0,
				(Float) parameters.get("fy"), (Float) parameters.get("cy"), 0, 0, 1);
		return K;
	}

	public static Mat getDistCoeffs() {

		// convert List<Double> to double[] (hate that I have to do this)
		Double[] DistCoeffs = ((List<Double>) parameters.get("distCoeffs")).toArray(new Double[0]);
		double[] distCoeffs = Stream.of(DistCoeffs).mapToDouble(Double::doubleValue).toArray();

		Mat distMat = new Mat(1, distCoeffs.length, CvType.CV_64FC1);
		distMat.put(0, 0, distCoeffs);
		return distMat;
	}

}
