package initializers;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.DMatch;

import Jama.Matrix;
import lumoslam.Keyframe;
import lumoslam.Map;
import lumoslam.MapPoint;
import lumoslam.ORBMatcher;
import lumoslam.Photogrammetry;
import runtimevars.Parameters;
import toolbox.Utils;
import types.Correspondence2D2D;
import types.Dbl;
import types.ImageData;
import types.InitializationException;
import types.Point3D;
import types.Pose;

public class ButtInitializer implements Initializer {

	public static int FRAME_CAP = 60;
	int MATCHING_THRESHOLD = 20;

	Map map = null;
	ImageData imageData = null;
	String referenceTitle = "";

	public ButtInitializer() {

	}

	public ButtInitializer(Map map) {
		this.map = map;
	}

	public List<Correspondence2D2D> registerData(long frameNum, String frameTitle, ImageData imageData,
			List<Correspondence2D2D> outAllCorrespondences) {

		if (frameNum == 0) {
			this.imageData = imageData;
			this.referenceTitle = frameTitle;
		} else {

			// go backwards through each registered frame and look for sufficient matches
			boolean matchFound = false;

			// get correspondences
			ImageData referenceFrame = this.imageData;
			List<Correspondence2D2D> correspondences = ORBMatcher.matchDescriptors(
					referenceFrame.getKeypoints().toList(), referenceFrame.getMostRecentDescriptors(),
					imageData.getKeypoints().toList(), imageData.getDescriptors(), MATCHING_THRESHOLD);

			// get median length of correspondences
			List<Double> corrLengths = new ArrayList<Double>();
			for (int j = 0; j < correspondences.size(); j++) {
				corrLengths.add(correspondences.get(j).getLength());
			}
			Dbl avg = new Dbl(0);
			Dbl stdDev = new Dbl(0);
			Dbl median = new Dbl(0);
			Utils.basicStats(corrLengths, avg, stdDev, median);

			Utils.pl("correspondences.size(): " + correspondences.size());
			Utils.pl("median: " + median);
			Utils.pl("stdDev: " + stdDev);

			// if there are enough correspondences and the median length of them is > 50,
			// attempt to initialize
			if (median.getValue() > 50 && imageData.getKeypoints().rows() >= 100) {
				try {
					matchFound = true;
					if (stdDev.getValue() > 15) {
						return initializeFundamental(frameNum, this.referenceTitle, frameTitle, referenceFrame,
								imageData, correspondences);
					} else {
						return initializeHomography(frameNum, this.referenceTitle, frameTitle, referenceFrame,
								imageData, correspondences);
					}

				} catch (Exception e) {
					matchFound = false;
				}
			}

		}

		return ORBMatcher.matchDescriptors(this.imageData.getKeypoints().toList(),
				this.imageData.getMostRecentDescriptors(), imageData.getKeypoints().toList(),
				imageData.getDescriptors(), MATCHING_THRESHOLD);

	}

	public List<Correspondence2D2D> initializeHomography(long frameNum, String frameTitle0, String frameTitle1,
			ImageData imageData0, ImageData imageData1, List<Correspondence2D2D> outAllCorrespondences)
			throws InitializationException {

		double initializationReprojectionError = Parameters.<Double>get("initializationReprojectionError");

		// estimate a pose
		List<Boolean> inlierMap = new ArrayList<Boolean>();
		Dbl numWithHighParallax = new Dbl(0);
		Pose pose = Photogrammetry.SFMHomographyEstimate(outAllCorrespondences, inlierMap, numWithHighParallax);

		// if not enough iniers or not enough points with high parallax, throw exception
		// and stop trying to initialize with this frame
		if (numWithHighParallax.getValue() < 50 || inlierMap.stream().filter(val -> val).count() < 40) {
			throw new InitializationException();
		}

		// register the initial keyframe
		Keyframe referenceFrame = this.map.registerInitialKeyframe(imageData0, frameNum, frameTitle0);

		// get correspondences against reference keyframe (List<DMatch>)
		long start = System.currentTimeMillis();
		List<DMatch> matches = ORBMatcher.matchDescriptors(referenceFrame, imageData1.getDescriptors());
		long end = System.currentTimeMillis();
		Utils.pl("Matching time: " + (end - start) + "ms");

		// initialize list of MapPoints based on the above match list
		List<MapPoint> matchedMapPoints = new ArrayList<MapPoint>();
		for (int i = 0; i < imageData1.getDescriptors().rows(); i++) {
			matchedMapPoints.add(null);
		}
		for (int i = 0; i < matches.size(); i++) {
			matchedMapPoints.set(matches.get(i).queryIdx, referenceFrame.getMapPoints().get(matches.get(i).trainIdx));
		}

		// triangulate all matched points
		Matrix E = pose.getHomogeneousMatrix();
		Matrix I = Matrix.identity(4, 4);

		Matrix K = Parameters.getK();

		// (inlier correspondences)
		List<Correspondence2D2D> registeredCorrespondences = new ArrayList<Correspondence2D2D>();

		synchronized (this.map) {
			for (int i = 0; i < outAllCorrespondences.size(); i++) {

				// ignore the outliers
				if (!inlierMap.get(i))
					continue;

				Correspondence2D2D c = outAllCorrespondences.get(i);
				Dbl parallax = new Dbl(0);
				Dbl err0 = new Dbl(0);
				Dbl err1 = new Dbl(0);
				Dbl w0 = new Dbl(0);
				Dbl w1 = new Dbl(0);
				Matrix point = Photogrammetry.triangulate(E, I, outAllCorrespondences.get(i), parallax, err1, err0, w1,
						w0);

				// if not enough parallax, skip
				if (parallax.getValue() < 1.0) {
					continue;
				}

				// check if point is behind cameras
				if (w0.getValue() < 0 || w1.getValue() < 0) {
					continue;
				}

				// if bad reprojection error, skip
				if (err0.getValue() > initializationReprojectionError
						|| err1.getValue() > initializationReprojectionError) {
					continue;
				}

				Point3D point3D = new Point3D(point.get(0, 0), point.get(1, 0), point.get(2, 0));
				MapPoint mapPoint = matchedMapPoints.get(matches.get(i).queryIdx);
				mapPoint.setPoint(point3D, frameNum);
				this.map.registerMapPoint(mapPoint);
				this.map.registerPoint(point3D);
				registeredCorrespondences.add(outAllCorrespondences.get(i));
			}
			// construct new keyframe
			this.map.registerNewKeyframe(frameNum, frameTitle1, pose, imageData1.getKeypoints(),
					imageData1.getDescriptors(), matchedMapPoints);
			this.map.setInitialized(true);
		}

		return registeredCorrespondences;

	}

	public List<Correspondence2D2D> initializeFundamental(long frameNum, String frameTitle0, String frameTitle1,
			ImageData imageData0, ImageData imageData1, List<Correspondence2D2D> outAllCorrespondences)
			throws InitializationException {

		double initializationReprojectionError = Parameters.<Double>get("initializationReprojectionError");

		// estimate a pose
		List<Boolean> inlierMap = new ArrayList<Boolean>();
		Dbl numWithHighParallax = new Dbl(0);
		Dbl cheirality = new Dbl(0);
		Pose pose = Photogrammetry.SFMFundamentalMatrixEstimate(outAllCorrespondences, inlierMap, numWithHighParallax,
				cheirality);

		// if not enough iniers or not enough points with high parallax, throw exception
		// and stop trying to initialize with this frame
		if (numWithHighParallax.getValue() < 50 || inlierMap.stream().filter(val -> val).count() < 40) {
			throw new InitializationException();
		}

		// register the initial keyframe
		Keyframe referenceFrame = this.map.registerInitialKeyframe(imageData0, frameNum, frameTitle0);

		// get correspondences against reference keyframe (List<DMatch>)
		long start = System.currentTimeMillis();
		List<DMatch> matches = ORBMatcher.matchDescriptors(referenceFrame, imageData1.getDescriptors());
		long end = System.currentTimeMillis();
		Utils.pl("Matching time: " + (end - start) + "ms");

		// initialize list of MapPoints based on the above match list
		List<MapPoint> matchedMapPoints = new ArrayList<MapPoint>();
		for (int i = 0; i < imageData1.getDescriptors().rows(); i++) {
			matchedMapPoints.add(null);
		}
		for (int i = 0; i < matches.size(); i++) {
			matchedMapPoints.set(matches.get(i).queryIdx, referenceFrame.getMapPoints().get(matches.get(i).trainIdx));
		}

		// triangulate all matched points
		Matrix E = pose.getHomogeneousMatrix();
		Matrix I = Matrix.identity(4, 4);

		Matrix K = Parameters.getK();

		// (inlier correspondences)
		List<Correspondence2D2D> registeredCorrespondences = new ArrayList<Correspondence2D2D>();

		synchronized (this.map) {
			for (int i = 0; i < outAllCorrespondences.size(); i++) {

				// ignore the outliers
				if (!inlierMap.get(i))
					continue;

				Correspondence2D2D c = outAllCorrespondences.get(i);
				Dbl parallax = new Dbl(0);
				Dbl err0 = new Dbl(0);
				Dbl err1 = new Dbl(0);
				Dbl w0 = new Dbl(0);
				Dbl w1 = new Dbl(0);
				Matrix point = Photogrammetry.triangulate(E, I, outAllCorrespondences.get(i), parallax, err1, err0, w1,
						w0);

				// if not enough parallax, skip
				if (parallax.getValue() < 1.0) {
					continue;
				}

				// check if point is behind cameras
				if (w0.getValue() < 0 || w1.getValue() < 0) {
					continue;
				}

				// if bad reprojection error, skip
				if (err0.getValue() > initializationReprojectionError
						|| err1.getValue() > initializationReprojectionError) {
					continue;
				}

//				Utils.pl(point.get(0, 0) + ", " + point.get(1, 0) + ", " + point.get(2, 0));
				Point3D point3D = new Point3D(point.get(0, 0), point.get(1, 0), point.get(2, 0));
				MapPoint mapPoint = matchedMapPoints.get(matches.get(i).queryIdx);
				mapPoint.setPoint(point3D, frameNum);
				this.map.registerMapPoint(mapPoint);
				this.map.registerPoint(point3D);
				registeredCorrespondences.add(outAllCorrespondences.get(i));
			}
			// construct new keyframe
			this.map.registerNewKeyframe(frameNum, frameTitle1, pose, imageData1.getKeypoints(),
					imageData1.getDescriptors(), matchedMapPoints);
			this.map.setInitialized(true);
		}

		return registeredCorrespondences;

	}

}
