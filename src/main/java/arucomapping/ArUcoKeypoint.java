package arucomapping;

import java.util.ArrayList;
import java.util.List;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;

public class ArUcoKeypoint {

	// IDs are formatted as <number>-<corner (0-3)>
	// i.e. all corners of marker 7 would be id = 7-0, 7-1, 7-2, 7-3
	public String id = "";
	public double locationX = 0;
	public double locationY = 0;

	public ArUcoKeypoint() {

	}

	public ArUcoKeypoint(String id, double locationX, double locationY) {
		this.id = id;
		this.locationX = locationX;
		this.locationY = locationY;
	}

	public String toString() {
		return String.format("ArUco marker ID: %6s at %5.4f, %5.4f", this.id, this.locationX, this.locationY);
	}

	public static List<ArUcoKeypoint> getArUcoMarkerLocations(Mat frame) {

		Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_6X6_1000);
		DetectorParameters dParameters = DetectorParameters.create();
		Mat ids = new Mat();
		List<Mat> corners = new ArrayList<Mat>();
		List<Mat> rejectedImgPoints = new ArrayList<Mat>();

		// detect markers
		Aruco.detectMarkers(frame, dictionary, corners, ids, dParameters, rejectedImgPoints);

		// convert results to ArUcoKeypoint list
		List<ArUcoKeypoint> keypoints = new ArrayList<ArUcoKeypoint>();
		if (ids.rows() > 0) {
			int[] idArr = new int[ids.rows()];
			ids.get(0, 0, idArr);
			for (int i = 0; i < idArr.length; i++) {
				Mat cornerData = corners.get(i);
				for (int j = 0; j < 4; j++) {
					String arucoID = idArr[i] + "-" + j;
					double[] loc = cornerData.get(0, j);
					ArUcoKeypoint kp = new ArUcoKeypoint(arucoID, loc[0], loc[1]);
					keypoints.add(kp);
				}
			}
		}

		return keypoints;

	}

}