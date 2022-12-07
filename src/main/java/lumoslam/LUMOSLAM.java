package lumoslam;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;

import arucomapping.ArUcoKeypoint;
import arucomapping.MarkerMPCorrespondence;
import buffers.Buffer;
import buffers.QueuedBuffer;
import buffers.SingletonBuffer;
import mock.MockEnvironment;
import placerecognition.BoWVector;
import runtimevars.Parameters;
import toolbox.Timer;
import toolbox.Utils;
import types.Callback;
import types.Correspondence2D2D;
import types.Dbl;
import types.FramePack;
import types.ImageData;
import types.PipelineOutput;
import types.Pose;
import types.TrackingException;

public class LUMOSLAM extends SLAMSystem<Map> {

	long targetFrametime = 30;
	int frameNum = 0;
//	MockPointData mock = new MockPointData();
	MockEnvironment mock = new MockEnvironment();

	Map map = new Map();
	Tracker tracker = new Tracker(this.map);
	MapOptimizer mapOptimizer = new MapOptimizer(this.map);

	long lastRecordedPose = 0;
	Pose latestPose = new Pose();
	Pose poseVelocity = new Pose();

	// list of the partition names found in the last successful track
	List<String> lastPartitions = new ArrayList<String>();

	double motionModelUncertaintyFactor = 1.2;

	// define that the list of keyframes is passed to the callbacks on completion
	protected void callFinishedCallback(Callback<Map> cb) {
		cb.callback(this.map);
	}

	Thread poseEstimationThread = new Thread() {
		@Override
		public void run() {
			try {
				mainloop();
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	};

	// REFACTOR : not sure we need the thread group anymore
	Thread mapOptimizationThread = new Thread(this.mapOptimizer.threadGroup, "mapOptimizationThread") {
		@Override
		public void run() {
			try {
				mapOptimizer.start();
			} catch (InterruptedException e) {
				Utils.pl("Map Optimization thread interrupted.");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	};

	Thread markerMapOptimizationThread = new Thread("markerMapOptimizationThread") {
		@Override
		public void run() {
			try {
				if (!Parameters.<Boolean>get("usingMarkers")) {
					return;
				}
				map.getMarkerMap().getOptimizer().start();
			} catch (InterruptedException e) {
				Utils.pl("Marker Map Optimization thread interrupted.");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	};

	public LUMOSLAM() {
		super(new QueuedBuffer<FramePack>(), new SingletonBuffer<PipelineOutput>());
	}

	public LUMOSLAM(Buffer<FramePack> inputBuffer, Buffer<PipelineOutput> outputBuffer) {
		super(inputBuffer, outputBuffer);
	}

	@Override
	public void start() {
		this.poseEstimationThread.start();
	}

	@Override
	public void stop() {
		this.poseEstimationThread.interrupt();
	}

	public void mainloop() throws Exception {
		boolean keepGoing = true;
		while (keepGoing) {

			long start = System.currentTimeMillis();

			Utils.pl("================  FRAME " + this.frameNum + " ================");

			PipelineOutput po = new PipelineOutput();

//			// mock
//			if (this.mock.totalFrames <= this.frameNum) {
//				keepGoing = false;
//				this.mapOptimizer.threadGroup.interrupt();
//				continue;
//			}
//
//			// mock - processing the image and extracting features
//			List<Byte> imageBufferGrey = new ArrayList<Byte>();
//			List<Byte> imageBufferRGB = new ArrayList<Byte>();
//			ImageData processedImage = this.mock.getImageData(this.frameNum, imageBufferGrey, imageBufferRGB);

			// real data
			// get the next frame
			FramePack currentFrame = this.inputBuffer.getNext();

			// undistort radial distortion
			if (Parameters.<List<Double>>get("distCoeffs").size() > 0 && currentFrame != null) {
				currentFrame.undistrortRawAndProcessedFrames(Parameters.getKMat(), Parameters.getDistCoeffs());
			}

			if (currentFrame == null) {
				keepGoing = false;
				this.mapOptimizer.threadGroup.interrupt();
				if (Parameters.<Boolean>get("usingMarkers")) {
					this.map.getMarkerMap().getOptimizer().threadGroup.interrupt();
				}
				this.triggerFinished();
				continue;
			}

			// transform image and feature matching (REAL DATA)
			ImageData processedImage = new ImageData(currentFrame.getProcessedFrame());
//			processedImage.autoContrast();
			ORBExtractor extractor = new ORBExtractor(1.2f, ImageData.orb.getNLevels(), ImageData.orb.getPatchSize());
			extractor.setKeypoints(processedImage);
			processedImage.computeFeatures();

			// initialize variables
			List<Correspondence2D2D> correspondences = new ArrayList<Correspondence2D2D>();
			List<Correspondence2D2D> prunedCorrespondences = new ArrayList<Correspondence2D2D>();
			List<Correspondence2D2D> outlierCorrespondences = new ArrayList<Correspondence2D2D>();
			List<Correspondence2D2D> moCorrespondences = new ArrayList<Correspondence2D2D>();
			Pose pose = new Pose();
			List<MapPoint> mapPointPerDescriptor = new ArrayList<MapPoint>();
			List<MapPoint> prunedCorrespondenceMapPoints = new ArrayList<MapPoint>();
			List<MapPoint> outlierCorrespondenceMapPoints = new ArrayList<MapPoint>();
			List<Correspondence2D2D> untriangulatedCorrespondences = new ArrayList<Correspondence2D2D>();
			List<MapPoint> untriangulatedMapPoints = new ArrayList<MapPoint>();
			List<Correspondence2D2D> includedPrunedCorrespondences = new ArrayList<Correspondence2D2D>();
			List<MapPoint> includedPrunedCorrespondenceMapPoints = new ArrayList<MapPoint>();
			List<KeyPoint> trackedKeypoints = new ArrayList<KeyPoint>();

			synchronized (this.map.loopClosureLock) {
				try {
					if (!this.map.isInitialized()) {

						// register keypoints and descriptors with initializer until it can initialize
						// the map
						prunedCorrespondences = this.map.getInitializer().registerData(this.frameNum,
								currentFrame.getFrameTitle(), processedImage, correspondences);

						// if it initialized this frame, get the map's new keyframe as the current pose
						if (this.map.isInitialized()) {
							pose = new Pose(this.map.getCurrentKeyframe().getPose());
							this.mapOptimizer.fullBundleAdjustment(100);
							this.mapOptimizer.pruneOutliers(this.map.getKeyframes());
							this.mapOptimizer.fullBundleAdjustment(100);
							this.mapOptimizationThread.start();
							this.markerMapOptimizationThread.start();
						}

					} else {

						// perform routine PnP pose estimation
						Dbl inlierRate = new Dbl(0);
						Dbl numTracked = new Dbl(0);
						int numMatches = tracker.trackMovement(processedImage.getKeypoints(),
								processedImage.getDescriptors(), correspondences, prunedCorrespondences,
								prunedCorrespondenceMapPoints, outlierCorrespondences, outlierCorrespondenceMapPoints,
								mapPointPerDescriptor, untriangulatedCorrespondences, untriangulatedMapPoints, pose,
								inlierRate, numTracked);

						// register moving objects
						tracker.registerMovingObjects3(pose, prunedCorrespondences, prunedCorrespondenceMapPoints,
								outlierCorrespondences, outlierCorrespondenceMapPoints);

						// detect and localize moving objects
						Timer t = new Timer();
						tracker.trackMovingObjects(pose, processedImage.getKeypoints(),
								processedImage.getDescriptors());
						long molTime = t.stop();
						if (this.map.getMovingObjects().size() > 0) {
							Utils.pl("MO LOCALIZAITON TIME: " + molTime + "ms");
						}

						// point inclusion
						long s = System.currentTimeMillis();
						tracker.partitionedPointInclusion(processedImage.getKeypoints(),
								processedImage.getDescriptors(), mapPointPerDescriptor, includedPrunedCorrespondences,
								includedPrunedCorrespondenceMapPoints, pose);
						long e = System.currentTimeMillis();
						Utils.pl("\n\n\n\npoint inclusion time: " + (e - s) + "ms\n\n\n\n");

						// if heavily violating motion model, assume error
						if (this.map.getCurrentKeyframe().getFrameNum() - this.map.getLastLoopClosed() > 1
								&& Utils.poseDistance(pose,
										Utils.getPoseEstimate(this.lastRecordedPose, this.latestPose, this.poseVelocity,
												this.frameNum)) > 100
														* Math.pow(this.motionModelUncertaintyFactor,
																this.frameNum - this.lastRecordedPose)) {
							Utils.pl("MOTION MODEL VIOLATED: "
									+ Utils.poseDistance(pose, Utils.getPoseEstimate(this.lastRecordedPose,
											this.latestPose, this.poseVelocity, this.frameNum)));
							throw new TrackingException();
						}

						Utils.getTrackedKeypoints(processedImage.getKeypoints().toList(), mapPointPerDescriptor,
								trackedKeypoints);

						Dbl avg = new Dbl(0);
						Dbl stdDev = new Dbl(0);
						Dbl median = new Dbl(0);

						Utils.basicStats(correspondences.stream().map(c -> c.getLength()).collect(Collectors.toList()),
								avg, stdDev, median);

						// check for poor tracking
						if (inlierRate.getValue() < 0.5 || numTracked.getValue() < 10) {
							Utils.pl("inlierRate: " + inlierRate);
							Utils.pl("numTracked: " + numTracked);
							throw new TrackingException();
						}

						// check for cheirality violation
						HashMap<MapPoint, Double> cheiralityValues = new HashMap<MapPoint, Double>();
						Photogrammetry.getCheiralityMap(mapPointPerDescriptor, pose, cheiralityValues);
						double numInFront = cheiralityValues.values().stream().filter(v -> v > 0).count();
						if (numInFront / cheiralityValues.values().size() < 0.5) {
							Utils.pl("CHEIRALITY VIOLATION");
							Utils.pl("numInFront: " + numInFront);
							Utils.pl("cheiralityValues.values().size(): " + cheiralityValues.values().size());
							Utils.pl("ratio: " + numInFront / cheiralityValues.values().size());
							throw new TrackingException();
						}

//						Utils.pl("Pair BA: ");
//						this.mapOptimizer.pairBundleAdjustment(pose, this.map.getCurrentKeyframe().getPose(),
//								prunedCorrespondenceMapPoints, prunedCorrespondences, 10);

						// triangulate untriangulated points
						if (untriangulatedCorrespondences.size() > 0) {

							// prune correspondences with epipolar search
							Photogrammetry.epipolarPrune(untriangulatedCorrespondences, untriangulatedMapPoints, pose,
									this.map.getCurrentKeyframe().getPose());

							// triangulate remaining correspondences
							Photogrammetry.triangulateUntrackedMapPoints(this.frameNum, pose,
									untriangulatedCorrespondences, untriangulatedMapPoints, this.map);

							// pair-wise BA
							Utils.pl("triangulation BA:");
							this.mapOptimizer.pairBundleAdjustment(pose, this.map.getCurrentKeyframe().getPose(),
									prunedCorrespondenceMapPoints, prunedCorrespondences, 10);

						}

						// if starting to lose tracking, generate new keyframe
						// if (starting to lose tracking OR we see more than 100 non-registered
						// keypoints)
						Utils.pl("prunedCorrespondences.size(): " + prunedCorrespondences.size());
						Utils.pl("numMatches: " + numMatches);
						Utils.pl("numTracked: " + numTracked);
						Utils.pl("untriangulatedMapPoints.size(): " + untriangulatedMapPoints.size());

						if ((median.getValue() > 0.035 * Parameters.<Integer>get("width")
								|| correspondences.size() < 70)) { // 0.035
							// when moving back to
							// true pipeline
							Utils.pl(
									"--- registering  new keyframe ------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
							Keyframe keyframe = this.map.registerNewKeyframe(this.frameNum,
									currentFrame.getFrameTitle(), pose, processedImage.getKeypoints(),
									processedImage.getDescriptors(), mapPointPerDescriptor);

							// register observations for markers if they are being used
							if (Parameters.<Boolean>get("usingMarkers")) {
								List<ArUcoKeypoint> markerKeypoints = ArUcoKeypoint
										.getArUcoMarkerLocations(processedImage.getImage());
								List<MarkerMPCorrespondence> markerMPCs = this.map.getMarkerMap()
										.getMapPointCorrespondences(markerKeypoints);
								this.map.getMarkerMap().registerObservations(keyframe, markerMPCs);
							}

						}

					}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					// TEMP: Identify best keyframes from BoW vector
					BoWVector vector = new BoWVector();
					Mat bestDescriptors = BoWVector.getBestDescriptors(processedImage.getKeypoints(),
							processedImage.getDescriptors(), 0.5);
					vector.compute(bestDescriptors);

					class KeyframeScore {
						public Keyframe keyframe;
						public double score;
					}

					List<KeyframeScore> KFScores = new ArrayList<KeyframeScore>();
					for (int i = 0; i < this.map.getKeyframes().size(); i++) {
						Keyframe kf = this.map.getKeyframes().get(i);
						KeyframeScore kfScore = new KeyframeScore();
						kfScore.keyframe = kf;
						kfScore.score = vector.getScore(kf.getBowVector());
						KFScores.add(kfScore);
					}

//				List<Keyframe> closestKFs = KFScores.stream().filter(kfs -> kfs.score >= 0.25).map(kfs -> kfs.keyframe)
//						.collect(Collectors.toList());
//				List<Keyframe> closestKFs = KFScores.stream().sorted(new Comparator<KeyframeScore>() {
//					public int compare(KeyframeScore kfs1, KeyframeScore kfs2) {
//						return kfs1.score - kfs2.score < 0 ? 1 : -1;
//					}
//				}).map(kfs -> kfs.keyframe).collect(Collectors.toList()).subList(0, 10);
					List<Keyframe> closestKFs = new ArrayList<Keyframe>();

					po.closestKeyframes = closestKFs;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

					// set pipeline output
					po.pose = pose;

					this.poseVelocity = Utils.getPoseDifference(pose, latestPose);
					this.latestPose = pose;
					this.lastRecordedPose = this.frameNum;
					po.trackingSuccessful = true;

					// if some error happened, use the old pose for output and send signal to the
					// output
				} catch (Exception e) {

					po.pose = this.latestPose;
					po.trackingSuccessful = false;
					Utils.pl("====  TRACKING FAILED FOR FRAME " + this.frameNum + "  ====");
					e.printStackTrace();

				}
			}

			// pipeline output
			po.frameNum = this.frameNum++;
			po.frameTitle = currentFrame.getFrameTitle();

			// image data
//			// mock
//			po.rawFrameBuffer = Utils.byteArray(imageBufferRGB);
//			po.processedFrameBuffer = Utils.byteArray(imageBufferGrey);
			// real data
			po.rawFrameBuffer = currentFrame.getRawFrameBuffer();
			po.processedFrameBuffer = processedImage.getBuffer();
			po.cameras = this.map.getCameras();
			po.points = this.map.getAllPoints();
			po.keyframes = this.map.getKeyframes();
//			if (this.map.getCurrentKeyframe() != null && this.map.getCurrentKeyframe().getPreviousKeyframe() != null) {
//				po.numKeyframes = (int) (this.map.getCurrentKeyframe().getFrameNum()
//						- this.map.getCurrentKeyframe().getPreviousKeyframe().getFrameNum());
//			}
			po.numKeyframes = this.map.getKeyframes().size();
			po.currentKeyframe = this.map.getCurrentKeyframe();

			po.features = processedImage.getKeypoints().toList();
			po.numFeatures = po.features.size();
			po.correspondences = correspondences;
			po.prunedCorrespondences = prunedCorrespondences;
			po.untriangulatedCorrespondences = untriangulatedCorrespondences;
			po.trackedKeypoints = trackedKeypoints;

			po.map = this.map;
			po.mapPointsByPartition = map.getMapPointsByPartition();
			po.trackedMapPoints = Utils.compressList(mapPointPerDescriptor);
			po.movingObjects = this.map.getMovingObjects();
			Utils.pl("total moving objects: " + po.movingObjects.size());

			po.markerMap = this.map.getMarkerMap();

			long end = System.currentTimeMillis();
			po.fps = 1000 / (end - start + 1);
			Utils.pl("frametime: " + (end - start) + "ms");
			Utils.pl("framerate: " + po.fps);

			try {
				long sleepTime = (end - start) < this.targetFrametime ? this.targetFrametime - (end - start) : 0;
//				if (this.frameNum > 590) {
//					sleepTime = 500;
//				}
				Thread.sleep(sleepTime);
			} catch (Exception e) {
			}

			outputBuffer.push(po);

		}
	}

}
