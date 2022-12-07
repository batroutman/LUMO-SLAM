package mock;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import types.Transformation;

// moving objects are entities that contain a collection of motion models at various points in time 
public class MovingObject {

	// a collection of motion models, indexed by the first frame they start on. it
	// is assumed that a motion model ends after the next begins. for example, this
	// map may contain {(0, Static...), (60, Linear...)}
	private HashMap<Long, MotionModel> motionModels = new HashMap<Long, MotionModel>();

	// continually updated sorted (ascending) list of motionModels.keySet();
	private List<Long> sortedMotionModelFrames = new ArrayList<Long>();

	// continually updated map of initial transformations to speed up computation of
	// current transformation. for example, if motionModels contains {(0,
	// Static...), (60, Linear...), (120, Linear...)}, then this map will need
	// entries for the second and third entry in motionModels. i.e., {(60,
	// Transformation), (120, Transformation)}
	private HashMap<Long, Transformation> startingPoints = new HashMap<Long, Transformation>();

	// initialized with a static motion model at frame 0
	public MovingObject() {
		this.addMotionModel(0L, new StaticMotionModel());
	}

	// give the frame number, output matrix T such that the absolute location of the
	// points is given by TX
	public Transformation getTransformation(long frameNum) {

		// find the appropriate index in motionModels
		Long frameStart = this.sortedMotionModelFrames.get(0);
		boolean found = false;
		for (int i = this.sortedMotionModelFrames.size() - 1; !found && i >= 0; i--) {
			if (this.sortedMotionModelFrames.get(i) <= frameNum) {
				found = true;
				frameStart = this.sortedMotionModelFrames.get(i);
			}
		}

		// select starting point (identity if beginning/no starting point)
		Transformation initialTransformation = this.startingPoints.get(frameStart);
		if (initialTransformation == null) {
			initialTransformation = new Transformation();
		}

		// combine transformations and return
		Transformation relativeTransformation = this.motionModels.get(frameStart)
				.getTransformation(frameNum - frameStart);

		Transformation fullTransformation = new Transformation();
		fullTransformation.transform(initialTransformation);
		fullTransformation.transform(relativeTransformation);

		return fullTransformation;

	}

	public void addMotionModel(Long frameStart, MotionModel motionModel) {
		this.motionModels.put(frameStart, motionModel);
		this.updateSortedMotionModelFrames();
		this.computeAllStartingPoints();
	}

	public void clearMotionModels() {
		this.motionModels.clear();
		this.updateSortedMotionModelFrames();
		this.computeAllStartingPoints();
	}

	// refresh entries for startingPoints
	public void computeAllStartingPoints() {

		this.startingPoints.clear();

		// compute total length of each motion model
		List<Long> intervals = new ArrayList<Long>();
		for (int i = 1; i < this.sortedMotionModelFrames.size(); i++) {
			intervals.add(this.sortedMotionModelFrames.get(i) - this.sortedMotionModelFrames.get(i - 1));
		}

		// compute starting points
		List<Transformation> startingPointsList = new ArrayList<Transformation>();
		for (int i = 0; i < intervals.size(); i++) {
			Transformation sp = new Transformation();
			if (startingPointsList.size() > 0) {
				sp = new Transformation(startingPointsList.get(i - 1));
			}

			Transformation relativeTransformation = motionModels.get(this.sortedMotionModelFrames.get(i))
					.getTransformation(intervals.get(i));

			sp.transform(relativeTransformation);
			startingPointsList.add(sp);
		}

		// load starting points
		for (int i = 0; i < startingPointsList.size(); i++) {
			this.startingPoints.put(this.sortedMotionModelFrames.get(i + 1), startingPointsList.get(i));
		}

	}

	public void updateSortedMotionModelFrames() {
		this.sortedMotionModelFrames.clear();
		List<Long> frames = new ArrayList<Long>(this.motionModels.keySet());
		Collections.sort(frames);
		this.sortedMotionModelFrames.addAll(frames);
	}

}
