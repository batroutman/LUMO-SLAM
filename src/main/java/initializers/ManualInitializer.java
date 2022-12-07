package initializers;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;

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
import types.Point3D;
import types.Pose;

public class ManualInitializer implements Initializer {

	ImageData referenceData = null;
	String referenceTitle = "";
	Map map = null;

	public ManualInitializer() {

	}

	public ManualInitializer(Map map) {
		this.map = map;
	}

	public List<Correspondence2D2D> registerData(long frameNum, String frameTitle, ImageData imageData,
			List<Correspondence2D2D> outAllCorrespondences) {
		if (frameNum == 0) {
			this.referenceData = imageData;
			this.referenceTitle = frameTitle;
		} else if (frameNum == 29) {
			return this.initialize(frameNum, frameTitle, imageData, outAllCorrespondences);
		}
		return ORBMatcher.matchDescriptors(this.referenceData.getKeypoints().toList(),
				this.referenceData.getDescriptors(), imageData.getKeypoints().toList(), imageData.getDescriptors());

	}

	public List<Correspondence2D2D> initialize(long frameNum, String frameTitle, ImageData imageData,
			List<Correspondence2D2D> outAllCorrespondences) {
		Utils.pl("Attempting to initialize...");

		double initializationReprojectionError = Parameters.<Double>get("initializationReprojectionError");

		// construct reference keyframe
		Keyframe referenceFrame = this.map.registerInitialKeyframe(this.referenceData, frameNum, this.referenceTitle);

		// get correspondences against reference keyframe (: List<MapPoint>)
		long start = System.currentTimeMillis();
		List<DMatch> matches = ORBMatcher.matchDescriptors(referenceFrame, imageData.getDescriptors());
		long end = System.currentTimeMillis();
		Utils.pl("Matching time: " + (end - start) + "ms");
		List<MapPoint> matchedMapPoints = new ArrayList<MapPoint>();
		for (int i = 0; i < imageData.getDescriptors().rows(); i++) {
			matchedMapPoints.add(null);
		}
		for (int i = 0; i < matches.size(); i++) {
			matchedMapPoints.set(matches.get(i).queryIdx, referenceFrame.getMapPoints().get(matches.get(i).trainIdx));
		}

		// construct correspondence list (List<Correspondence2D2D>)
		outAllCorrespondences.addAll(this.inferCorrespondences(referenceFrame, imageData, matches));

		// get sfm estimate from correspondences
		List<Boolean> inlierMap = new ArrayList<Boolean>();
		Pose pose = Photogrammetry.SFMFundamentalMatrixEstimate(outAllCorrespondences, inlierMap, new Dbl(0),
				new Dbl(0));

		// triangulate all matched points
		Matrix E = pose.getHomogeneousMatrix();
		Matrix I = Matrix.identity(4, 4);

		Matrix K = Parameters.getK();

		List<Correspondence2D2D> registeredCorrespondences = new ArrayList<Correspondence2D2D>();

		// TODO: investigate this reprojection error. Are erroneous correspondences even
		// correlated with higher reprojection error when triangulated with the HZ
		// approach?
		float reprojError = 50f;
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
		}

		// construct new keyframe
		this.map.registerNewKeyframe(frameNum, frameTitle, pose, imageData.getKeypoints(), imageData.getDescriptors(),
				matchedMapPoints);

		this.map.setInitialized(true);

		return registeredCorrespondences;

	}

	public List<Correspondence2D2D> inferCorrespondences(Keyframe referenceFrame, ImageData currentData,
			List<DMatch> currentMatches) {

		List<Correspondence2D2D> correspondences = new ArrayList<Correspondence2D2D>();

		List<KeyPoint> referenceKeypoints = referenceFrame.getKeypoints().toList();
		List<KeyPoint> currentKeypoints = currentData.getKeypoints().toList();

		for (DMatch match : currentMatches) {
			Correspondence2D2D c = new Correspondence2D2D();
			c.setX0(referenceKeypoints.get(match.trainIdx).pt.x);
			c.setY0(referenceKeypoints.get(match.trainIdx).pt.y);
			c.setX1(currentKeypoints.get(match.queryIdx).pt.x);
			c.setY1(currentKeypoints.get(match.queryIdx).pt.y);
			correspondences.add(c);
		}

		return correspondences;

	}

	public ImageData getReferenceData() {
		return referenceData;
	}

	public void setReferenceData(ImageData referenceData) {
		this.referenceData = referenceData;
	}

	public Map getMap() {
		return map;
	}

	public void setMap(Map map) {
		this.map = map;
	}

}
