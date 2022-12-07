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

public class SimpleInitializer implements Initializer {

	public static int FRAME_CAP = 60;

	Map map = null;
	List<ImageData> imageDatas = new ArrayList<ImageData>();

	public SimpleInitializer() {

	}

	public SimpleInitializer(Map map) {
		this.map = map;
	}

	public List<Correspondence2D2D> registerData(long frameNum, String frameTitle, ImageData imageData,
			List<Correspondence2D2D> outAllCorrespondences) {

		if (frameNum == 0) {
			imageDatas.add(imageData);
		} else {

			// go backwards through each registered frame and look for sufficient matches
			boolean matchFound = false;
			for (int i = 0; i < this.imageDatas.size() && !matchFound; i++) {

				// get correspondences
				ImageData referenceFrame = imageDatas.get(i);
				List<Correspondence2D2D> correspondences = ORBMatcher.matchDescriptors(
						referenceFrame.getKeypoints().toList(), referenceFrame.getDescriptors(),
						imageData.getKeypoints().toList(), imageData.getDescriptors());

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

				// if there are enough correspondences and the median length of them is >= 30,
				// attempt to initialize
				if (correspondences.size() >= 30 && median.getValue() >= 30) {
					try {
						matchFound = true;
						return initialize(frameNum, "0", frameTitle, referenceFrame, imageData, correspondences);
					} catch (Exception e) {
						matchFound = false;
						e.printStackTrace();
					}
				}

			}
			if (this.imageDatas.size() >= FRAME_CAP) {
				this.imageDatas.remove(0);
			}
		}

		return ORBMatcher.matchDescriptors(this.imageDatas.get(0).getKeypoints().toList(),
				this.imageDatas.get(0).getDescriptors(), imageData.getKeypoints().toList(), imageData.getDescriptors());

	}

	public List<Correspondence2D2D> initialize(long frameNum, String frameTitle0, String frameTitle1,
			ImageData imageData0, ImageData imageData1, List<Correspondence2D2D> outAllCorrespondences)
			throws InitializationException {

		// estimate a pose
		List<Boolean> inlierMap = new ArrayList<Boolean>();
		Dbl numWithHighParallax = new Dbl(0);
		Dbl cheirality = new Dbl(0);
		Pose pose = Photogrammetry.SFMFundamentalMatrixEstimate(outAllCorrespondences, inlierMap, numWithHighParallax,
				cheirality);

		Utils.pl("NUMBER OF CORRESPONDENCES: " + outAllCorrespondences.size());
		Utils.pl("numWithHighParallax: " + numWithHighParallax);
		Utils.pl("cheirality: " + cheirality);

		// if not enough iniers or there are not enough points with parallax > 1.0,
		// throw exception and stop trying to initialize with this frame
		if (numWithHighParallax.getValue() < 50 || inlierMap.stream().filter(val -> val).count() < 20
				|| (double) inlierMap.stream().filter(val -> val).count() / inlierMap.size() < 0.6) {
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

				// if parallax is too low, throw out
				if (parallax.getValue() < 1.0) {
					continue;
				}

				// check if point is behind cameras
				if (w0.getValue() < 0 || w1.getValue() < 0) {
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

	public Map getMap() {
		return map;
	}

	public void setMap(Map map) {
		this.map = map;
	}

	public List<ImageData> getImageDatas() {
		return imageDatas;
	}

	public void setImageDatas(List<ImageData> imageDatas) {
		this.imageDatas = imageDatas;
	}

}
