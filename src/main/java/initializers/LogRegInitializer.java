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

public class LogRegInitializer implements Initializer {

	int MAX_FRAMES = 60;
	ImageData referenceFrame = null;
	String referenceTitle = "";

	Map map = null;

	// synapses (last one is bias)
//	double[] funSynapses = { 13.3096, -45.3710, -39.0948, -2.3385, -13.2645, 17.8734, 2.8865, 41.3986, 15.9338,
//			-25.8702, -11.2718, -10.3142, -14.7470, -25.0620, 14.5940, -4.1055, -4.4091, -4.5082, -1.8191, -5.3409,
//			-15.2998, -17.9376, -9.0944, 27.0736 };
//	double[] essSynapses = { 13.0164, -38.8185, -36.4260, -2.1292, -8.2050, 19.2759, 4.1697, 41.9846, 17.0715, -27.3653,
//			-15.5801, -6.0046, -14.9766, -24.4136, 11.6138, -3.7307, -4.0196, -3.5096, -1.3691, -3.8592, -18.9556,
//			-5.7228, -6.2250, 21.2665 };
//	double[] homSynapses = { 12.1375, -34.3721, -22.1990, -10.9383, 14.1869, -10.7557, -4.3175, 12.6953, -4.1871,
//			12.7619, -4.1710, 26.2160, 6.4884, -17.6290, -16.7008, -0.6676, -2.6420, 0.1798, -3.5334, -1.6524, -2.4385,
//			-6.5192, -12.2233, -8.7873 };
//	double[] rotSynapses = { -3.4095, -119.8966, -48.1568, -5.6690, -15.1910, -9.0810, -2.5055, -4.0016, 3.7151,
//			18.2536, 8.4372, -9.3176, 6.8650, 7.6018, -10.0858, -1.7192, -3.0965, -3.4424, -1.7384, -6.4752, -3.8989,
//			-3.4673, -3.9367, 18.5941 };

//	double[] funSynapses = { 10.8047, 125.1486, 50.9052, 5.2209, 10.1422, 10.7325, -1.9719, -6.9588, -9.2347, -15.8191,
//			-12.6815, 5.0158, -13.0794, -2.3131, 3.2515, 1.8012, 3.7345, 3.7553, 3.6720, 5.7241, 5.3894, 2.7991, 3.6406,
//			4.2549 };

	// retrained on orb features (original labeling)
	double[] funSynapses = { 0.46228474378585815, 8.893998146057129, -2.762807607650757, -17.84194564819336,
			-8.040422439575195, 6.887745380401611, 3.083099842071533, 9.034113883972168, 10.363031387329102,
			2.5257654190063477, 0.921903133392334, 9.637781143188477, -4.29905366897583, 0.29179680347442627,
			-0.8382972478866577, -2.563039779663086, -1.2157484292984009, -1.130225658416748, -2.2886500358581543,
			-2.8437001705169678, -6.523902416229248, -6.032505989074707, -1.6368565559387207, -10.071742057800293 };

	// orbslam labeling
//	double[] funSynapses = { -4.436557292938232, 6.460002422332764, 0.17521941661834717, 7.352662086486816,
//			10.551030158996582, 6.095883846282959, 5.720611095428467, -15.045144081115723, -13.51401138305664,
//			6.7081756591796875, 0.9001498222351074, 2.9221508502960205, -1.0109927654266357, 1.7835814952850342,
//			-5.886326789855957, -1.8139647245407104, 0.10106164962053299, 1.8214293718338013, -1.4670120477676392,
//			4.406700611114502, 0.1603112369775772, -0.7510892152786255, 2.8721420764923096, 3.2118005752563477 };

	public LogRegInitializer() {

	}

	public LogRegInitializer(Map map) {
		this.map = map;
	}

	public List<Correspondence2D2D> registerData(long frameNum, String frameTitle, ImageData imageData,
			List<Correspondence2D2D> outAllCorrespondences) {

		if (frameNum % MAX_FRAMES == 0) {
			this.referenceFrame = imageData;
			this.referenceTitle = frameTitle;
		} else {
			List<Correspondence2D2D> correspondences = ORBMatcher.matchDescriptors(
					referenceFrame.getKeypoints().toList(), referenceFrame.getDescriptors(),
					imageData.getKeypoints().toList(), imageData.getDescriptors());

			CorrespondenceSummary cs = new CorrespondenceSummary(correspondences);
			double[] data = cs.getArray();
			cs.printData();

			if (this.evaluate(data, this.funSynapses)) {
				// initialize
				try {
					return this.initialize(frameNum, this.referenceTitle, frameTitle, this.referenceFrame, imageData,
							correspondences);
				} catch (Exception e) {
					e.printStackTrace();
				}

			}

		}

		return ORBMatcher.matchDescriptors(this.referenceFrame.getKeypoints().toList(),
				this.referenceFrame.getDescriptors(), imageData.getKeypoints().toList(), imageData.getDescriptors());
	}

	public boolean evaluate(double[] data, double[] synapses) {
		double output = 0;
		for (int i = 0; i < data.length; i++) {
			output += data[i] * synapses[i];
		}

		double activated = 1 / (1 + Math.exp(-output));

		Utils.pl("activated: " + activated);

		return activated > 0.5;
	}

	public List<Correspondence2D2D> initialize(long frameNum, String frameTitle0, String frameTitle1,
			ImageData imageData0, ImageData imageData1, List<Correspondence2D2D> outAllCorrespondences)
			throws InitializationException {

		double initializationReprojectionError = Parameters.<Double>get("initializationReprojectionError");

		// estimate a pose
		List<Boolean> inlierMap = new ArrayList<Boolean>();
		Dbl numWithHighParallax = new Dbl(0);
		Dbl cheirality = new Dbl(0);
		Pose pose = Photogrammetry.SFMFundamentalMatrixEstimate(outAllCorrespondences, inlierMap, numWithHighParallax,
				cheirality);
		Utils.pl("numWithHighParallax: " + numWithHighParallax);
		Utils.pl("cheirality: " + cheirality);

		// if not enough iniers or not enough points with high parallax, throw exception
		// and stop trying to initialize with this frame
		if (numWithHighParallax.getValue() < 50
				|| (double) inlierMap.stream().filter(val -> val).count() / inlierMap.size() < 0.9
				|| cheirality.getValue() / outAllCorrespondences.size() < 0.9) {
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

				// if low parallax, throw out
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
				this.map.partitionMapPoint(mapPoint);
				this.map.getUncheckedMapPoints().add(mapPoint);
				registeredCorrespondences.add(outAllCorrespondences.get(i));
			}
			// construct new keyframe
			this.map.registerNewKeyframe(frameNum, frameTitle1, pose, imageData1.getKeypoints(),
					imageData1.getDescriptors(), matchedMapPoints);
			this.map.setInitialized(true);
		}

		return registeredCorrespondences;

	}

	public class CorrespondenceSummary {

		double MAX_DISPARITY = Math.sqrt(Math.pow(Parameters.<Integer>get("width").doubleValue(), 2)
				+ Math.pow(Parameters.<Integer>get("height").doubleValue(), 2));

		public int numCorrespondences = 0;
		public double meanDisparity = 0;
		public double stdDevDisparity = 0;
		public double minX0 = 999999999;
		public double maxX0 = 0;
		public double minY0 = 999999999;
		public double maxY0 = 0;
		public double minX1 = 999999999;
		public double maxX1 = 0;
		public double minY1 = 999999999;
		public double maxY1 = 0;
		public double rangeX0 = 0;
		public double rangeY0 = 0;
		public double rangeX1 = 0;
		public double rangeY1 = 0;

		// rotation bins (north, northeast, east, etc.)
		public int binN = 0;
		public int binNE = 0;
		public int binE = 0;
		public int binSE = 0;
		public int binS = 0;
		public int binSW = 0;
		public int binW = 0;
		public int binNW = 0;

		public CorrespondenceSummary() {
		}

		public CorrespondenceSummary(List<Correspondence2D2D> correspondences) {
			this.evaluate(correspondences);
		}

		// return a double array of the input data (normalized bins)
		public double[] getArray() {

			int width = Parameters.<Integer>get("width");
			int height = Parameters.<Integer>get("height");

			double[] features = new double[24];
			features[0] = this.numCorrespondences / 300.0;
			features[1] = this.meanDisparity / MAX_DISPARITY;
			features[2] = this.stdDevDisparity / MAX_DISPARITY;
			features[3] = this.minX0 / width;
			features[4] = this.maxX0 / width;
			features[5] = this.minY0 / height;
			features[6] = this.maxY0 / height;
			features[7] = this.minX1 / width;
			features[8] = this.maxX1 / width;
			features[9] = this.minY1 / height;
			features[10] = this.maxY1 / height;
			features[11] = this.rangeX0 / width;
			features[12] = this.rangeY0 / height;
			features[13] = this.rangeX1 / width;
			features[14] = this.rangeY1 / height;

			// get magnitude of rotation bins
			double mag = Math.sqrt(Math.pow(binN, 2) + Math.pow(binNE, 2) + Math.pow(binE, 2) + Math.pow(binSE, 2)
					+ Math.pow(binS, 2) + Math.pow(binSW, 2) + Math.pow(binW, 2) + Math.pow(binNW, 2));

			features[15] = this.binN / mag;
			features[16] = this.binNE / mag;
			features[17] = this.binE / mag;
			features[18] = this.binSE / mag;
			features[19] = this.binS / mag;
			features[20] = this.binSW / mag;
			features[21] = this.binW / mag;
			features[22] = this.binNW / mag;

			// bias placeholder
			features[23] = 1;

			return features;
		}

		public void evaluate(List<Correspondence2D2D> correspondences) {

			long start = System.currentTimeMillis();
			this.numCorrespondences = correspondences.size();

			List<Double> disparities = new ArrayList<Double>();
			double sum = 0;

			for (int i = 0; i < correspondences.size(); i++) {
				Correspondence2D2D c = correspondences.get(i);

				// handle rotation data
				this.incrementRotationBin(this.getAngle(c));

				// handle min and max
				this.minX0 = Math.min(this.minX0, c.getX0());
				this.minY0 = Math.min(this.minY0, c.getY0());
				this.minX1 = Math.min(this.minX1, c.getX1());
				this.minY1 = Math.min(this.minY1, c.getY1());
				this.maxX0 = Math.max(this.maxX0, c.getX0());
				this.maxY0 = Math.max(this.maxY0, c.getY0());
				this.maxX1 = Math.max(this.maxX1, c.getX1());
				this.maxY1 = Math.max(this.maxY1, c.getY1());

				double disparity = Math.sqrt(Math.pow(c.getX0() - c.getX1(), 2) + Math.pow(c.getY0() - c.getY1(), 2));
				sum += disparity;
				disparities.add(disparity);

			}

			// range
			this.rangeX0 = maxX0 - minX0;
			this.rangeY0 = maxY0 - minY0;
			this.rangeX1 = maxX1 - minX1;
			this.rangeY1 = maxY1 - minY1;

			// mean
			this.meanDisparity = sum / this.numCorrespondences;

			// get standard deviation
			double sumSqDev = 0;
			for (int i = 0; i < disparities.size(); i++) {
				double disparity = disparities.get(i);
				sumSqDev += Math.pow(disparity - this.meanDisparity, 2);
			}
			this.stdDevDisparity = Math.sqrt(sumSqDev / this.numCorrespondences);

			long end = System.currentTimeMillis();
//			Utils.pl("correspondence summary time: " + (end - start) + "ms");

		}

		public double getAngle(Correspondence2D2D c) {

			// get x and y values
			double x = c.getX1() - c.getX0();
			double y = -(c.getY1() - c.getY0());

			// get base angle
			double angle = Math.atan2(y, x);

			return angle;

		}

		public double incrementRotationBin(double angle) {

			double newAngle = angle >= 0 ? angle + 0.3926990816987 : angle - 0.3926990816987;
			double index = (int) (newAngle / 0.7853981633974);

			if (index == 0) {
				this.binE++;
			} else if (index == 1) {
				this.binNE++;
			} else if (index == 2) {
				this.binN++;
			} else if (index == 3) {
				this.binNW++;
			} else if (index >= 4) {
				this.binW++;
			} else if (index == -1) {
				this.binSE++;
			} else if (index == -2) {
				this.binS++;
			} else if (index == -3) {
				this.binSW++;
			} else if (index <= -4) {
				this.binW++;
			}

			return index;

		}

		public void printData() {
			Utils.pl("numCorrespondences: " + numCorrespondences);
			Utils.pl("meanDisparity: " + meanDisparity);
			Utils.pl("stdDevDisparity: " + stdDevDisparity);
			Utils.pl("minX0: " + minX0);
			Utils.pl("maxX0: " + maxX0);
			Utils.pl("minY0: " + minY0);
			Utils.pl("maxY0: " + maxY0);
			Utils.pl("rangeX0: " + rangeX0);
			Utils.pl("rangeY0: " + rangeY0);
			Utils.pl("minX1: " + minX1);
			Utils.pl("maxX1: " + maxX1);
			Utils.pl("minY1: " + minY1);
			Utils.pl("maxY1: " + maxY1);
			Utils.pl("rangeX1: " + rangeX1);
			Utils.pl("rangeY1: " + rangeY1);
			Utils.pl("binN: " + binN);
			Utils.pl("binNE: " + binNE);
			Utils.pl("binE: " + binE);
			Utils.pl("binSE: " + binSE);
			Utils.pl("binS: " + binS);
			Utils.pl("binSW: " + binSW);
			Utils.pl("binW: " + binW);
			Utils.pl("binNW: " + binNW);
		}

		public String stringify() {
			String output = "";

			output += this.numCorrespondences + ",";
			output += this.meanDisparity + ",";
			output += this.stdDevDisparity + ",";
			output += this.minX0 + ",";
			output += this.maxX0 + ",";
			output += this.minY0 + ",";
			output += this.maxY0 + ",";
			output += this.rangeX0 + ",";
			output += this.rangeY0 + ",";
			output += this.minX1 + ",";
			output += this.maxX1 + ",";
			output += this.minY1 + ",";
			output += this.maxY1 + ",";
			output += this.rangeX1 + ",";
			output += this.rangeY1 + ",";
			output += this.binN + ",";
			output += this.binNE + ",";
			output += this.binE + ",";
			output += this.binSE + ",";
			output += this.binS + ",";
			output += this.binSW + ",";
			output += this.binW + ",";
			output += this.binNW + "\n";

			return output;
		}

	}

}
