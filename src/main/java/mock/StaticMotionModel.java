package mock;

import types.Transformation;

// non-moving motion model
public class StaticMotionModel implements MotionModel {

	Transformation transformation = new Transformation();

	public StaticMotionModel() {

	}

	public StaticMotionModel(Transformation transformation) {
		this.transformation = transformation;
	}

	public Transformation getTransformation(long numSteps) {
		return this.transformation;
	}

	public Transformation getTransformation() {
		return transformation;
	}

	public void setTransformation(Transformation transformation) {
		this.transformation = transformation;
	}

}
