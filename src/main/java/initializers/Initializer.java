package initializers;

import java.util.List;

import types.Correspondence2D2D;
import types.ImageData;

public interface Initializer {

	// give a new frame to the initializer and have it try to initialize
	// the output is the list of inlier correspondences
	// outAllCorrespondences is a list of all found keypoint matches
	public List<Correspondence2D2D> registerData(long frameNum, String frameTitle, ImageData imageData,
			List<Correspondence2D2D> outAllCorrespondences);
}
