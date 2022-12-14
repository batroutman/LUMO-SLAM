package loopclosure;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.opencv.core.DMatch;
import org.opencv.core.Point;
import org.opencv.core.Point3;

import boofcv.abst.geo.bundle.PruneStructureFromSceneMetric;
import boofcv.abst.geo.bundle.SceneObservations;
import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.alg.geo.bundle.cameras.BundlePinhole;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;
import lumoslam.Keyframe;
import lumoslam.Map;
import lumoslam.MapOptimizer;
import lumoslam.MapPoint;
import lumoslam.ORBMatcher;
import lumoslam.Observation;
import lumoslam.Photogrammetry;
import placerecognition.BoWVector;
import runtimevars.Parameters;
import toolbox.Timer;
import toolbox.Utils;
import types.Correspondence2D2D;
import types.Dbl;
import types.Point3D;
import types.Pose;

public class LoopClosure {

	protected MapOptimizer mapOptimizer = null;
	protected Map map = null;
	protected Keyframe lastKeyframeChecked = null;
	protected int pauseCounter = 0;

	private LoopClosure() {

	}

	public LoopClosure(Map map, MapOptimizer mapOptimizer) {
		this.map = map;
		this.mapOptimizer = mapOptimizer;
	}

	// run one iteration of loop closure
	public void checkLoopClosure() {

		int KF_PADDING = 10;
		int NUM_DEEP_CHECKS = 10;
		double MIN_BOW_SCORE = 0.2;
		int MATCH_THRESHOLD = 20; // 30
		int MIN_MATCHES_REQ = 50; // 70
		int LOOP_DETECTION_PAUSE_AMOUNT = 30;

		// find the next keyframe after the last one checked
		if (this.lastKeyframeChecked == null) {
			if (this.map.getKeyframes().size() > 0) {
				this.lastKeyframeChecked = this.map.getKeyframes().get(0);
			} else {
				return;
			}
		}
		Keyframe referenceKeyframe = this.lastKeyframeChecked.getNextKeyframe();
		if (referenceKeyframe == null) {
			return;
		}

		// update lastKeyframeChecked
		referenceKeyframe.setCheckedForLoopClosure(true);
		this.lastKeyframeChecked = referenceKeyframe;
		this.pauseCounter--;

		// if loop has been detected recently, skip
		if (this.pauseCounter > 0) {
			return;
		}

		// gather keyframes previous to this reference keyframe (that are N keyframes
		// away)
		List<Keyframe> keyframes = new ArrayList<Keyframe>(this.map.getKeyframes());
		int currentKFIndex = keyframes.indexOf(referenceKeyframe);
		if (currentKFIndex > KF_PADDING) {
			keyframes = keyframes.subList(0, currentKFIndex - KF_PADDING);
		} else {
			return;
		}

		// get BoW scores for these keyframes
		class KeyframeScore {
			public Keyframe keyframe;
			public double score;
			public List<DMatch> matches;
		}
		BoWVector vector = referenceKeyframe.getBowVector();
		List<KeyframeScore> KFScores = new ArrayList<KeyframeScore>();
		for (int i = 0; i < keyframes.size(); i++) {
			Keyframe kf = keyframes.get(i);
			KeyframeScore kfScore = new KeyframeScore();
			kfScore.keyframe = kf;
			kfScore.score = vector.getScore(kf.getBowVector());
			KFScores.add(kfScore);
		}

		// get BoW vector for previous keyframe
		KeyframeScore previousKFS = new KeyframeScore();
		previousKFS.keyframe = referenceKeyframe.getPreviousKeyframe();
		previousKFS.score = vector.getScore(previousKFS.keyframe.getBowVector());

		// sort keyframes by highest (best) BoW scores
		List<KeyframeScore> sortedKFScores = KFScores.stream().sorted(new Comparator<KeyframeScore>() {
			public int compare(KeyframeScore kfs1, KeyframeScore kfs2) {
				return kfs1.score - kfs2.score < 0 ? 1 : -1;
			}
		}).collect(Collectors.toList());

		// for the top M keyframes, attempt ORB feature matching (unguided)
		List<KeyframeScore> KFSsToCheck = sortedKFScores
				.subList(0, sortedKFScores.size() >= NUM_DEEP_CHECKS ? NUM_DEEP_CHECKS : sortedKFScores.size()).stream()
				.filter(kfs -> kfs.score / previousKFS.score >= MIN_BOW_SCORE).collect(Collectors.toList());

		for (int i = 0; i < KFSsToCheck.size(); i++) {
			KFSsToCheck.get(i).matches = ORBMatcher.matchDescriptors(KFSsToCheck.get(i).keyframe.getDescriptors(),
					referenceKeyframe.getDescriptors(), MATCH_THRESHOLD);
		}

		// if one of these keyframes achieves a good number of matches (~100?), consider
		// this a loop
		boolean loopFound = false;
		for (int i = 0; i < KFSsToCheck.size() && !loopFound; i++) {

			// validate that the matches aren't of moving object points, and do not
			// merge map points of moving object points
			Keyframe kf = KFSsToCheck.get(i).keyframe;
			List<DMatch> nonMOMatches = KFSsToCheck.get(i).matches.stream()
					.filter(match -> referenceKeyframe.getMapPoints().get(match.queryIdx).getAssociatedObject() == null
							&& kf.getMapPoints().get(match.trainIdx).getAssociatedObject() == null)
					.collect(Collectors.toList());
//			nonMOMatches.forEach(match -> Utils.pl("trainIdx: " + match.trainIdx + "\t\tqueryIdx: " + match.queryIdx));

			if (nonMOMatches.size() >= MIN_MATCHES_REQ) {
				loopFound = true;

				Utils.pl("================    LOOP DETECTED    ================");

				this.pauseCounter = LOOP_DETECTION_PAUSE_AMOUNT;

				// for visualization, note that these frames are part of loop
				this.map.getLoopDetectedKeyframes().add(referenceKeyframe);
				this.map.getLoopDetectedKeyframes().add(KFSsToCheck.get(i).keyframe);

				// close loop on this keyframe and reference frame
				synchronized (this.mapOptimizer.baLock) {
					synchronized (this.map.loopClosureLock) {
						synchronized (this.mapOptimizer.principalDescriptorLock) {
							synchronized (this.mapOptimizer.pointPrunerLock) {

								Utils.pl("Closing loop...");
								Timer t = new Timer();
								this.closeLoop2Step(kf, referenceKeyframe, nonMOMatches);
								long millis = t.stop();
								Utils.pl("Loop closed: " + millis + "ms");

							}
						}
					}
				}

			}
		}

	}

	public static class KFPEdge {
		public Point3D point; // point to optimize
		public Keyframe keyframe; // keyframe the point is constrained by
		public Point3D targetTransformedPoint; // tranformed point - how the point relates to this keyframe
		public Pose originalPose; // copy of original pose as reference
	}

	/**
	 * Merges common keypoints from danglingKF to groundTruthKF and closes the loop
	 * between them by first optimizing the error in the similarity transform
	 * between keyframes and then re-triangulating affected map points
	 * 
	 * @param groundTruthKF First keyframe in the loop.
	 * @param danglingKF    Last keyframe in loop.
	 * @param nonMOMatches  List of the matches between groundTruthKF and
	 *                      danglingKF, omitting any map points that belong to
	 *                      moving objects
	 */
	public void closeLoop2Step(Keyframe groundTruthKF, Keyframe danglingKF, List<DMatch> nonMOMatches) {

		int NUM_GROUND_TRUTH_KFS = 15;

		// register cancellation of all BA
		this.map.setLoopBeingClosed(true);

		Utils.pl("Before merging: " + danglingKF.validMapPointMapping());

		// merge matching points
		List<MapPoint> commonMapPoints = new ArrayList<MapPoint>();
		Utils.pl("danglingKF: " + danglingKF);
		for (DMatch match : nonMOMatches) {
			this.map.mergeMapPoint(groundTruthKF.getMapPoints().get(match.trainIdx),
					danglingKF.getMapPoints().get(match.queryIdx));
			commonMapPoints.add(groundTruthKF.getMapPoints().get(match.trainIdx));
		}

		Utils.pl("After merging: " + danglingKF.validMapPointMapping());

		// build full window of keyframes to adjust (including some number of original
		// keyframes)
		List<Keyframe> kfWindow = new ArrayList<Keyframe>();
		if (this.map.getLastLoopClosed() == -1) {
			kfWindow.add(groundTruthKF);
		} else {
			Keyframe lastLoopCloseKF = this.map.getKeyframes().stream()
					.filter(kf -> kf.getFrameNum() == this.map.getLastLoopClosed()).collect(Collectors.toList()).get(0);
			kfWindow.add(lastLoopCloseKF);
		}

		while (kfWindow.get(kfWindow.size() - 1) != this.map.getCurrentKeyframe()) {
			kfWindow.add(kfWindow.get(kfWindow.size() - 1).getNextKeyframe());
		}

		// extract keyframe-to-point edges for 2nd step optimization
		List<KFPEdge> kfpEdges = this.extractKFPEdges(kfWindow, commonMapPoints, 10);
		Utils.pl("kfpEdges.size(): " + kfpEdges.size());

		// optimize keyframe poses
		Timer t = new Timer();
		Dbl alpha = new Dbl(10);
		this.optimizeSimTransform(kfWindow, groundTruthKF, danglingKF, commonMapPoints, alpha, 100);
		long optimizationTime = t.stop();
		Utils.pl("optimization time: " + optimizationTime + "ms");

		// optimize the points affected by this loop closure
		Timer t1 = new Timer();
		this.optimizeKFPEdges(kfpEdges, alpha, 10);
		long kfpTime = t1.stop();
		Utils.pl("KFP Optimization time: " + kfpTime + "ms");

		// bundle adjustment to clean
		List<Keyframe> baWindow = kfWindow.subList(kfWindow.size() - 10, kfWindow.size());
		baWindow.add(groundTruthKF);
		groundTruthKF.getPose().setFixed(true);
		try {
			this.mapOptimizer.fixedWindowBundleAdjustment(baWindow, true, true, 10, false);
		} catch (Exception e) {
			e.printStackTrace();
		}
		groundTruthKF.getPose().setFixed(false);

		// register the latest loop-closed keyframe in the map (so BA knows if it needs
		// to cancel)
		this.map.setLoopBeingClosed(false);
		this.map.setLastLoopClosed(danglingKF.getFrameNum());

	}

	public List<KFPEdge> extractKFPEdges(List<Keyframe> kfWindow, List<MapPoint> commonMapPoints,
			int maxEdgesPerPoint) {

		// get all points in window
		HashMap<Point3D, List<Keyframe>> pointTable = new HashMap<Point3D, List<Keyframe>>();
		for (Keyframe kf : kfWindow) {
			for (MapPoint mp : kf.getMapPoints()) {
				if (mp.getPoint() != null) {
					if (pointTable.get(mp.getPoint()) == null) {
						pointTable.put(mp.getPoint(), new ArrayList<Keyframe>());
					}
					pointTable.get(mp.getPoint()).add(kf);
				}
			}
		}

		// remove commonMapPoints
		for (MapPoint mp : commonMapPoints) {
			Point3D p = mp.getPoint();
			if (p == null) {
				continue;
			}
			pointTable.remove(p);
		}

		List<KFPEdge> edges = new ArrayList<KFPEdge>();

		// build edges for each point
		for (Point3D p : pointTable.keySet()) {
			List<Keyframe> kfs = pointTable.get(p);
			for (int i = 0; i < kfs.size() && i <= maxEdgesPerPoint; i++) {

				KFPEdge edge = new KFPEdge();

				edge.point = p;
				edge.keyframe = kfs.get(i);
				edge.originalPose = new Pose(kfs.get(i).getPose());
				edge.targetTransformedPoint = edge.originalPose.transformPoint(edge.point);

				edges.add(edge);

			}
		}

		return edges;

	}

	public void optimizeKFPEdges(List<KFPEdge> edges, Dbl alpha, int epochs) {

		for (int i = 0; i < epochs; i++) {
			int j = 0;
			for (KFPEdge edge : edges) {
				this.adjustKFPEdge(edge, alpha, j == edges.size() / 2);
				j++;
			}
		}

	}

	public void adjustKFPEdge(KFPEdge edge, Dbl alpha, boolean printResidual) {

		Pose kfPose = edge.keyframe.getPose();
		Point3D currentPoint = edge.point;
		Point3D target = edge.targetTransformedPoint;

		// get point estimate
		Point3D estimatedPoint = kfPose.transformPoint(currentPoint);

		// get residual
		double diffX = estimatedPoint.getX() - target.getX();
		double diffY = estimatedPoint.getY() - target.getY();
		double diffZ = estimatedPoint.getZ() - target.getZ();

		// pre-compute common variables
		double TwoDiffX = 2 * diffX;
		double TwoDiffY = 2 * diffY;
		double TwoDiffZ = 2 * diffZ;

		double qw = kfPose.getQw();
		double qx = kfPose.getQx();
		double qy = kfPose.getQy();
		double qz = kfPose.getQz();

		double qwqw = qw * qw;
		double qxqx = qx * qx;
		double qyqy = qy * qy;
		double qzqz = qz * qz;

		double Twoqwqz = 2 * qw * qz;
		double Twoqxqy = 2 * qx * qy;
		double Twoqwqy = 2 * qw * qy;
		double Twoqxqz = 2 * qx * qz;
		double Twoqwqx = 2 * qw * qx;
		double Twoqyqz = 2 * qy * qx;

		// compute partials
		double dX = TwoDiffX * (qwqw + qxqx - qyqy - qzqz) + TwoDiffY * (Twoqwqz + Twoqxqy)
				+ TwoDiffZ * (-Twoqwqy + Twoqxqz);
		double dY = TwoDiffX * (-Twoqwqz + Twoqxqy) + TwoDiffY * (qwqw - qxqx + qyqy - qzqz)
				+ TwoDiffZ * (Twoqwqx + Twoqyqz);
		double dZ = TwoDiffX * (Twoqwqy + Twoqxqz) + TwoDiffY * (-Twoqwqx + Twoqyqz)
				+ TwoDiffZ * (qwqw - qxqx - qyqy + qzqz);

		// update point
		double newX = currentPoint.getX() - dX * alpha.getValue();
		double newY = currentPoint.getY() - dY * alpha.getValue();
		double newZ = currentPoint.getZ() - dZ * alpha.getValue();

		// commit changes
		currentPoint.setX(newX);
		currentPoint.setY(newY);
		currentPoint.setZ(newZ);

		if (printResidual) {
			Utils.pl("residual: " + diffX + ", " + diffY + ", " + diffZ);
		}

	}

	public static class Edge {
		public Keyframe kfi;
		public Keyframe kfj;
		public Pose ij;
		public boolean fixI = false;
		public boolean fixJ = false;
	}

	/**
	 * Minimizes the objective function (Kij - Ki * -Kj)^2, where -Kj is the
	 * inverted pose of keyframe j, Ki is the pose of keyframe i, and Kij is the
	 * similarity transform from Kj to Ki. (Ki, -Kj) pairs are generated by
	 * selecting neighboring keyframe pairs in loopChain. The initial values for Kij
	 * are locked for each pair and only the keyframe poses are optimized.
	 * Additionally, the dangling keyframe is relocalized and its similarity
	 * transform with the ground truth keyframe is calculated and added to the
	 * optimization (this is what constrains the optimization towards accuracy).
	 * 
	 * @param loopChain       ordered list of keyframes starting from the latest
	 *                        loop-closed keyframe or from the ground truth
	 *                        keyframe. NOTE: this list should NOT place
	 *                        groundTruthKF and danglingKF next to each other.
	 * @param groundTruthKF   the first keyframe of the loop
	 * @param danglingKF      the last keyframe of the loop
	 * @param commonMapPoints list of the newly-merged map points between danglingKF
	 *                        and groundTruthKF
	 * @param epochs          number of iterations to optimize with gradient descent
	 */
	public void optimizeSimTransform(List<Keyframe> loopChain, Keyframe groundTruthKF, Keyframe danglingKF,
			List<MapPoint> commonMapPoints, Dbl alpha, int epochs) {

		// build edges
		List<Edge> edges = new ArrayList<Edge>();
		for (int i = 0; i < loopChain.size() - 1; i++) {
			Edge edge = new Edge();
			edge.kfi = loopChain.get(i);
			edge.kfj = loopChain.get(i + 1);
			edge.ij = edge.kfi.getPose().similarityTransformFrom(edge.kfj.getPose());
			Utils.pl("sim transform: " + edge.ij.toString());
			edge.fixI = i == 0;
			edge.fixJ = false;
			edges.add(edge);
		}

		// peek a relocalization of danglingKF and capture the new similarity transform
		// against groundTruthKF. Use this to append an edge between danglingKF and
		// groundTruthKF with the new similarity transform
		List<Point3> point3s = new ArrayList<Point3>();
		List<Point> points = new ArrayList<Point>();
		List<Integer> inlierIndices = new ArrayList<Integer>();

		for (MapPoint mp : commonMapPoints) {
			Point3D point3D = mp.getPoint();
			if (point3D == null) {
				continue;
			}
			Point3 pt3 = new Point3(point3D.getX(), point3D.getY(), point3D.getZ());
			if (danglingKF.getMapPointIndices().get(mp) != null) {

				for (Integer index : danglingKF.getMapPointIndices().get(mp)) {
					Point pt = danglingKF.getKeypointsList().get(index).pt;
					point3s.add(pt3);
					points.add(pt);
				}
			}
		}

		// use the bundle adjuster to optimize the dangling pose (start at the position
		// of the ground truth pose to guarantee that the starting pose is reasonably
		// close to where it should be, in case the dangling keyframe is super far away)
		Pose newPose = new Pose(groundTruthKF.getPose());
		this.adjustPose(danglingKF, groundTruthKF, newPose, commonMapPoints);

		Utils.pl("newPose: " + newPose.toString());
		Utils.pl("newPose getQw(): " + newPose.getQw());

		Edge edge = new Edge();
		edge.kfi = danglingKF;
		edge.kfj = groundTruthKF;
		edge.ij = newPose.similarityTransformFrom(edge.kfj.getPose());

		Utils.pl("i (dangling): " + edge.kfi.getPose().toString());
		Utils.pl("j (groundTruth): " + edge.kfj.getPose().toString());
		Utils.pl("ij: " + edge.ij.toString());

		edge.fixJ = true;
		edges.add(edge);

		// optimize edges
		Pose currentSimTransform = danglingKF.getPose().similarityTransformFrom(groundTruthKF.getPose());
		double maxTranslation = Math.sqrt(Math.pow(currentSimTransform.getCx(), 2)
				+ Math.pow(currentSimTransform.getCy(), 2) + Math.pow(currentSimTransform.getCy(), 2));
		for (int i = 0; i < epochs; i++) {
//			Utils.pl("Epoch: " + i);
			for (int j = edges.size() - 1; j >= 0; j--) {
				adjustEdge(edges.get(j), alpha, maxTranslation);
			}
		}

	}

	public void adjustPose(Keyframe danglingKF, Keyframe groundTruthKF, Pose poseToModify,
			List<MapPoint> commonMapPoints) {

		List<MapPoint> matchedMapPoints = new ArrayList<MapPoint>();
		List<Correspondence2D2D> correspondences = new ArrayList<Correspondence2D2D>();
		for (MapPoint mp : commonMapPoints) {
			if (mp.getPoint() == null) {
				continue;
			}
			if (danglingKF.getMapPointIndices().get(mp) == null || groundTruthKF.getMapPointIndices().get(mp) == null) {
				continue;
			}
			Correspondence2D2D c = new Correspondence2D2D();
			Point groundTruthPt = groundTruthKF.getKeypointsList()
					.get(groundTruthKF.getMapPointIndices().get(mp).get(0)).pt;
			Point danglingPt = danglingKF.getKeypointsList().get(danglingKF.getMapPointIndices().get(mp).get(0)).pt;
			c.setX0(groundTruthPt.x);
			c.setY0(groundTruthPt.y);
			c.setX1(danglingPt.x);
			c.setY1(danglingPt.y);
			correspondences.add(c);
			matchedMapPoints.add(mp);
		}

		Utils.pl("correspondences: " + correspondences.size());

		try {
			this.mapOptimizer.pairBundleAdjustment(poseToModify, groundTruthKF.getPose(), matchedMapPoints,
					correspondences, 100, true, true);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public void adjustEdge(Edge edge, Dbl alpha, double maxTranslation) {

		double minAlpha = 0.00000001;

		Pose i = edge.kfi.getPose();
		Pose j = edge.kfj.getPose();
		Pose ij = edge.ij;

		// get inverted j
		Pose jInv = new Pose(j);
		jInv.setQw(-jInv.getQw());
		jInv.setCx(-jInv.getCx());
		jInv.setCy(-jInv.getCy());
		jInv.setCz(-jInv.getCz());

		// compute final pose
		Pose base = jInv.applyPose(i);

		// pre-compute quaternion errors
		double twoDiffQw = 2 * (base.getQw() - ij.getQw());
		double twoDiffQx = 2 * (base.getQx() - ij.getQx());
		double twoDiffQy = 2 * (base.getQy() - ij.getQy());
		double twoDiffQz = 2 * (base.getQz() - ij.getQz());

		// pre-compute translation errors
		double twoDiffCx = 2 * (base.getCx() - ij.getCx());
		double twoDiffCy = 2 * (base.getCy() - ij.getCy());
		double twoDiffCz = 2 * (base.getCz() - ij.getCz());

		// if not fixed, update poses
		if (!edge.fixI) {

			// calculate gradient
			double dQwI = twoDiffQw * (-j.getQw()) + twoDiffQx * (j.getQx()) + twoDiffQy * (j.getQy())
					+ twoDiffQz * (j.getQz());
			double dQxI = twoDiffQw * (-j.getQx()) + twoDiffQx * (-j.getQw()) + twoDiffQy * (-j.getQz())
					+ twoDiffQz * (j.getQy());
			double dQyI = twoDiffQw * (-j.getQy()) + twoDiffQx * (j.getQz()) + twoDiffQy * (-j.getQw())
					+ twoDiffQz * (-j.getQx());
			double dQzI = twoDiffQw * (-j.getQz()) + twoDiffQx * (-j.getQy()) + twoDiffQy * (j.getQx())
					+ twoDiffQz * (-j.getQw());

			double dCxI = twoDiffCx;
			double dCyI = twoDiffCy;
			double dCzI = twoDiffCz;

			// attempt to update pose i (adjust alpha if necessary)
			boolean keepGoing = true;
			while (keepGoing) {
				double newQw = i.getQw() - dQwI * alpha.getValue();
				double newQx = i.getQx() - dQxI * alpha.getValue();
				double newQy = i.getQy() - dQyI * alpha.getValue();
				double newQz = i.getQz() - dQzI * alpha.getValue();
				double newCx = i.getCx() - dCxI * alpha.getValue();
				double newCy = i.getCy() - dCyI * alpha.getValue();
				double newCz = i.getCz() - dCzI * alpha.getValue();

				Pose tempI = new Pose();
				tempI.setQw(newQw);
				tempI.setQx(newQx);
				tempI.setQy(newQy);
				tempI.setQz(newQz);
				tempI.setCx(newCx);
				tempI.setCy(newCy);
				tempI.setCz(newCz);
				tempI.normalize();

				// check if the translation change is surpassing the max expected
				double translation = Math.sqrt(Math.pow(i.getCx() - tempI.getCx(), 2)
						+ Math.pow(i.getCy() - tempI.getCy(), 2) + Math.pow(i.getCz() - tempI.getCz(), 2));

				// if error is less than before, commit and stop
				if (translation < maxTranslation) {
					i.setPose(tempI);
					keepGoing = false;
					continue;
				} else {
					// if alpha is already less than some threshold, stop
					if (alpha.getValue() < minAlpha) {
						keepGoing = false;
						Utils.pl("alpha passed threshold (" + alpha.getValue() + ")");
						continue;
					} else {
						// otherwise, lower alpha an try again
						alpha.setValue(alpha.getValue() * 0.9);
						Utils.pl("lowering alpha to " + alpha.getValue());
						continue;
					}
				}

			}
		}

		if (!edge.fixJ) {

			// calculate gradient
			double dQwJ = twoDiffQw * (-i.getQw()) + twoDiffQx * (-i.getQx()) + twoDiffQy * (-i.getQy())
					+ twoDiffQz * (-i.getQz());
			double dQxJ = twoDiffQw * (-i.getQx()) + twoDiffQx * (i.getQw()) + twoDiffQy * (i.getQz())
					+ twoDiffQz * (-i.getQy());
			double dQyJ = twoDiffQw * (-i.getQy()) + twoDiffQx * (-i.getQz()) + twoDiffQy * (i.getQw())
					+ twoDiffQz * (i.getQx());
			double dQzJ = twoDiffQw * (-i.getQz()) + twoDiffQx * (i.getQy()) + twoDiffQy * (-i.getQx())
					+ twoDiffQz * (i.getQw());

			double dCxJ = -twoDiffCx;
			double dCyJ = -twoDiffCy;
			double dCzJ = -twoDiffCz;

			// attempt to update pose i (adjust alpha if necessary)
			boolean keepGoing = true;
			while (keepGoing) {
				double newQw = j.getQw() - dQwJ * alpha.getValue();
				double newQx = j.getQx() - dQxJ * alpha.getValue();
				double newQy = j.getQy() - dQyJ * alpha.getValue();
				double newQz = j.getQz() - dQzJ * alpha.getValue();
				double newCx = j.getCx() - dCxJ * alpha.getValue();
				double newCy = j.getCy() - dCyJ * alpha.getValue();
				double newCz = j.getCz() - dCzJ * alpha.getValue();

				Pose tempJ = new Pose();
				tempJ.setQw(newQw);
				tempJ.setQx(newQx);
				tempJ.setQy(newQy);
				tempJ.setQz(newQz);
				tempJ.setCx(newCx);
				tempJ.setCy(newCy);
				tempJ.setCz(newCz);
				tempJ.normalize();

				// check if the translation change is surpassing the max expected
				double translation = Math.sqrt(Math.pow(j.getCx() - tempJ.getCx(), 2)
						+ Math.pow(j.getCy() - tempJ.getCy(), 2) + Math.pow(j.getCz() - tempJ.getCz(), 2));

				// if error is less than before, commit and stop
				if (translation < maxTranslation) {
					j.setPose(tempJ);
					keepGoing = false;
					continue;
				} else {
					// if alpha is already less than some threshold, stop
					if (alpha.getValue() < minAlpha) {
						keepGoing = false;
						Utils.pl("alpha passed threshold (" + alpha.getValue() + ")");
						continue;
					} else {
						// otherwise, lower alpha an try again
						alpha.setValue(alpha.getValue() * 0.9);
						Utils.pl("lowering alpha to " + alpha.getValue());
						continue;
					}
				}

			}

		}

	}

	public void retriangulateMapPoints(List<Keyframe> kfWindow, long frameNum) {

		// collect map points
		HashMap<MapPoint, Boolean> affectedMapPoints = new HashMap<MapPoint, Boolean>();
		for (Keyframe kf : kfWindow) {
			for (MapPoint mp : kf.getMapPoints()) {
				if (mp.getPoint() != null) {
					affectedMapPoints.put(mp, true);
				}
			}
		}

		// retriangulate map points
		int numRetriangulated = 0;
		for (MapPoint mp : affectedMapPoints.keySet()) {
			Dbl parallax = new Dbl(0);
			Dbl reprojErrorSecondary = new Dbl(0);
			Dbl reprojErrorPrimary = new Dbl(0);
			Dbl wSecondary = new Dbl(0);
			Dbl wPrimary = new Dbl(0);
			boolean retriangulated = Photogrammetry.triangulateMapPoint(mp, frameNum, parallax, reprojErrorSecondary,
					reprojErrorPrimary, wSecondary, wPrimary);
			numRetriangulated += retriangulated ? 1 : 0;
		}

		Utils.pl("Retriangulated map points: " + numRetriangulated + " / " + affectedMapPoints.keySet().size());
	}

	public void closeLoopBA(Keyframe groundTruthKF, Keyframe danglingKF, List<DMatch> nonMOMatches) {

		int NUM_GROUND_TRUTH_KFS = 15;

		// register cancellation of all BA
		this.map.setLoopBeingClosed(true);

		// merge matching points
		for (DMatch match : nonMOMatches) {
			this.map.mergeMapPoint(groundTruthKF.getMapPoints().get(match.trainIdx),
					danglingKF.getMapPoints().get(match.queryIdx));
		}

		// build full window of keyframes to adjust (including some number of original
		// keyframes)
		List<Keyframe> kfWindow = new ArrayList<Keyframe>();
		kfWindow.add(groundTruthKF);
		while (kfWindow.get(kfWindow.size() - 1) != danglingKF) {
			kfWindow.add(kfWindow.get(kfWindow.size() - 1).getNextKeyframe());
		}

		// // isolate the back half of the KF list and append the n ground truth frames
		List<Keyframe> kfWindowCopy = new ArrayList<Keyframe>(kfWindow);
		List<Keyframe> groundTruthKFs = new ArrayList<Keyframe>(kfWindow.subList(0, NUM_GROUND_TRUTH_KFS));
		List<Keyframe> halfwayWindow = new ArrayList<Keyframe>(kfWindow
				.subList(kfWindow.size() / 2 - NUM_GROUND_TRUTH_KFS, kfWindow.size() / 2 + NUM_GROUND_TRUTH_KFS));
		kfWindow = kfWindow.subList(kfWindow.size() / 2, kfWindow.size());
		kfWindow.addAll(groundTruthKFs);

		// repeatedly run windowed BA on keyframes, with most frames fixed in place
		for (int i = kfWindow.size(); i >= NUM_GROUND_TRUTH_KFS + 1; i--) {
			List<Keyframe> window = kfWindow.subList(i - NUM_GROUND_TRUTH_KFS - 1, i);
			List<Boolean> fixed = window.stream().map(kf -> true).collect(Collectors.toList());
			fixed.set(0, false);
			try {
				// perform fixed BA on subset of window
				this.fixedBA(kfWindow.subList(i - NUM_GROUND_TRUTH_KFS - 1, i), 10, fixed);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}

		// run windowed BA on the halfway point to patch up drift
		try {
			Utils.pl("Running BA on halfway window...");
			List<Boolean> fixed = IntStream.range(0, halfwayWindow.size())
					.mapToObj(i -> i == 0 || i == halfwayWindow.size() - 1).collect(Collectors.toList());
			this.fixedBA(halfwayWindow, 10, fixed);
			Utils.pl("BA complete.");
		} catch (Exception e) {
			e.printStackTrace();
		}

		// register the latest loop-closed keyframe in the map (so BA knows if it needs
		// to cancel)
		this.map.setLoopBeingClosed(false);
		this.map.setLastLoopClosed(danglingKF.getFrameNum());

	}

	// lock all keyframes except the first one and perform BA on the keyframe and
	// its points
	public void fixedBA(List<Keyframe> keyframes, int iterations, List<Boolean> fixed) throws Exception {

		SceneStructureMetric scene = new SceneStructureMetric(false);
		SceneObservations sceneObservations = new SceneObservations();

		HashMap<Keyframe, Integer> keyframeIndices = new HashMap<Keyframe, Integer>();
		List<Point3D> point3Ds = new ArrayList<Point3D>();
		List<MapPoint> capturedMapPoints = new ArrayList<MapPoint>();
		List<List<Keyframe>> keyframesPerPoint3D = new ArrayList<List<Keyframe>>();
		List<Observation> observations = new ArrayList<Observation>();
		List<Integer> observationToPoint3DIndex = new ArrayList<Integer>();

		// capture map data and load into BoofCV
		synchronized (this.map) {

			// collect common map points between keyframes
			HashMap<MapPoint, Boolean> commonMapPointTable = new HashMap<MapPoint, Boolean>();
			for (int i = 0; i < keyframes.size(); i++) {
				for (int j = 0; j < keyframes.get(i).getMapPoints().size(); j++) {
					MapPoint mp = keyframes.get(i).getMapPoints().get(j);
					if (mp.getAssociatedObject() != null) {
						continue;
					}
					if (commonMapPointTable.get(mp) == null) {
						commonMapPointTable.put(mp, false);
					} else {
						commonMapPointTable.put(mp, true);
					}
				}
			}
			List<MapPoint> mapPoints = new ArrayList<MapPoint>();
			for (MapPoint mp : commonMapPointTable.keySet()) {
				if (commonMapPointTable.get(mp)) {
					mapPoints.add(mp);
				}
			}

			// ---- capture data ---- //
			for (int i = 0; i < keyframes.size(); i++) {
				keyframeIndices.put(keyframes.get(i), i);
			}
			for (int i = 0; i < mapPoints.size(); i++) {
				MapPoint mp = mapPoints.get(i);
				if (mp.getPoint() == null) {
					continue;
				}
				HashMap<Keyframe, Boolean> associatedKeyframes = new HashMap<Keyframe, Boolean>();
				point3Ds.add(mp.getPoint());
				capturedMapPoints.add(mp);
				for (int j = 0; j < mp.getObservations().size(); j++) {

					// ensure that the observation is related to a keyframe in the window
					if (keyframeIndices.get(mp.getObservations().get(j).getKeyframe()) == null) {
						continue;
					}

					observations.add(mp.getObservations().get(j));
					observationToPoint3DIndex.add(point3Ds.size() - 1);
					associatedKeyframes.put(mp.getObservations().get(j).getKeyframe(), true);
				}
				List<Keyframe> associatedKeyframesList = new ArrayList<Keyframe>();
				associatedKeyframesList.addAll(associatedKeyframes.keySet());
				keyframesPerPoint3D.add(associatedKeyframesList);
			}

			// ---- load data into BoofCV ---- //
			scene.initialize(keyframes.size(), keyframes.size(), point3Ds.size());
			sceneObservations.initialize(keyframes.size());

			// load camera poses into scene
			BundlePinhole camera = new BundlePinhole();
			camera.fx = Parameters.<Float>get("fx").doubleValue();
			camera.fy = Parameters.<Float>get("fy").doubleValue();
			camera.cx = Parameters.<Float>get("cx").doubleValue();
			camera.cy = Parameters.<Float>get("cy").doubleValue();
			camera.skew = Parameters.<Float>get("s").doubleValue();
			for (int i = 0; i < keyframes.size(); i++) {
				Se3_F64 worldToCameraGL = new Se3_F64();
				ConvertRotation3D_F64.quaternionToMatrix(keyframes.get(i).getPose().getQw(),
						keyframes.get(i).getPose().getQx(), keyframes.get(i).getPose().getQy(),
						keyframes.get(i).getPose().getQz(), worldToCameraGL.R);
				worldToCameraGL.T.x = keyframes.get(i).getPose().getTx();
				worldToCameraGL.T.y = keyframes.get(i).getPose().getTy();
				worldToCameraGL.T.z = keyframes.get(i).getPose().getTz();
				scene.setCamera(i, true, camera);
				scene.setView(i, fixed.get(i), worldToCameraGL); // fix all keyframes that aren't first in list
				scene.connectViewToCamera(i, i);
			}

			// load points
			for (int i = 0; i < point3Ds.size(); i++) {
				float x = (float) point3Ds.get(i).getX();
				float y = (float) point3Ds.get(i).getY();
				float z = (float) point3Ds.get(i).getZ();
				scene.setPoint(i, x, y, z);
				for (int j = 0; j < keyframesPerPoint3D.get(i).size(); j++) {
					scene.getPoints().get(i).views.add(keyframeIndices.get(keyframesPerPoint3D.get(i).get(j)));
				}

			}

			// load observations
			for (int i = 0; i < observations.size(); i++) {
				Observation o = observations.get(i);
				int viewID = keyframeIndices.get(o.getKeyframe());
				int pointID = observationToPoint3DIndex.get(i);
				sceneObservations.getView(viewID).add(pointID, (float) o.getPoint().x, (float) o.getPoint().y);
			}

		}

		// ---- prune and bundle adjustment ---- //
		// omit top x% of observations with highest reprojection error
		PruneStructureFromSceneMetric pruner = new PruneStructureFromSceneMetric(scene, sceneObservations);

		// perform BA
		boolean BAIncomplete = true;
		int maxAttempts = 3;
		for (int i = 0; i <= maxAttempts && BAIncomplete; i++) {
			try {
				MapOptimizer.bundleAdjustScene(scene, sceneObservations, iterations);
				BAIncomplete = false;
			} catch (Exception e) {
				if (i >= maxAttempts) {
					throw e;
				}
				Utils.pl(
						"windowed BA failed, attempting to prune... !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
//				pruner.pruneObservationsByErrorRank(ITERATIVE_PRUNING_INLIER_THRESHOLD);
			}
		}

		// lock the map and update the points
		synchronized (this.map) {

			// load points from scene back into input
			for (int i = 0; i < scene.getPoints().size(); i++) {
				point3Ds.get(i).setX(scene.getPoints().get(i).getX());
				point3Ds.get(i).setY(scene.getPoints().get(i).getY());
				point3Ds.get(i).setZ(scene.getPoints().get(i).getZ());
				MapPoint mp = capturedMapPoints.get(i);
				if (mp.getAssociatedObject() != null) {
					this.map.unPartitionMapPoint(mp);
					this.map.partitionMapPoint(mp);
				}
			}

			// load poses from scene back into input
			for (int viewID = 0; viewID < keyframes.size(); viewID++) {
				Se3_F64 worldToView = scene.getViews().get(viewID).worldToView;
				Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(worldToView.getR(), null);
				q.normalize();
				Vector3D_F64 t = worldToView.getTranslation();
				keyframes.get(viewID).getPose().setQw(q.w);
				keyframes.get(viewID).getPose().setQx(q.x);
				keyframes.get(viewID).getPose().setQy(q.y);
				keyframes.get(viewID).getPose().setQz(q.z);
				keyframes.get(viewID).getPose().setT(t.x, t.y, t.z);
			}
		}

	}

}
