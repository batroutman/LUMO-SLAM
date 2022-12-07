package placerecognition;

import org.opencv.core.Mat;

public interface PlaceRecognitionVector<T extends PlaceRecognitionVector> {

	public void compute(Mat descriptors);

	public double getScore(T prv);

}
