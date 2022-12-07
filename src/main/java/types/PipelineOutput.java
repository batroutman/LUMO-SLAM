package types;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;

import arucomapping.ArUcoMap;
import lumoslam.Keyframe;
import lumoslam.Map;
import lumoslam.MapPoint;
import lumoslam.MovingModel;

// PipelineOutput is a type that packages all information that the desktop container may display for any single frame
public class PipelineOutput {

	// tracking state
	public boolean trackingSuccessful = true;

	//
	public boolean finalFrame = false;
	public double frameNum = 0;
	public String frameTitle = "";

	// performance and diagnostic data
	public double fps = 0;
	public int numKeyframes = 0;
	public int numFeatures = 0;
	public int numCorrespondences = 0;
	public boolean tracking = false;

	// pose params
	public Pose pose = new Pose();

	// frame data
	public Mat rawFrame = null;
	public Mat processedFrame = null;
	public byte[] rawFrameBuffer = null;
	public byte[] processedFrameBuffer = null;

	// processed frame data
	// all initial correspondences
	public List<Correspondence2D2D> correspondences = new ArrayList<Correspondence2D2D>();

	// correspondences after pruning
	public List<Correspondence2D2D> prunedCorrespondences = new ArrayList<Correspondence2D2D>();

	// matches that were not triangulated in time for tracking
	public List<Correspondence2D2D> untriangulatedCorrespondences = new ArrayList<Correspondence2D2D>();

	// keypoints that were matched to a triangulated map point
	public List<KeyPoint> trackedKeypoints = new ArrayList<KeyPoint>();

	// map points that were used in tracking
	public List<MapPoint> trackedMapPoints = new ArrayList<MapPoint>();

	public List<KeyPoint> features = new ArrayList<KeyPoint>();

	// map data
	public Map map = new Map();
	public List<Point3D> points = new ArrayList<Point3D>();
	public List<Pose> cameras = new ArrayList<Pose>();
	public List<Keyframe> keyframes = new ArrayList<Keyframe>();
	public Keyframe currentKeyframe = null;
	public HashMap<String, List<MapPoint>> mapPointsByPartition = new HashMap<String, List<MapPoint>>();
	public List<MovingModel> movingObjects = new ArrayList<MovingModel>();
	public ArUcoMap markerMap = new ArUcoMap();

	// temp
	public List<Keyframe> closestKeyframes = new ArrayList<Keyframe>();

}
