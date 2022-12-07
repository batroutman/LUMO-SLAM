package mock;

import types.Transformation;

public interface MotionModel {

	public Transformation getTransformation(long numSteps);

}
