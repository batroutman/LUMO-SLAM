package datarecording;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

import org.json.JSONObject;

import arucomapping.ArUcoMap;
import lumoslam.Map;
import runtimevars.Parameters;
import toolbox.Utils;
import types.Callback;
import types.Transformation;

public class MarkerSimTransformRecorder implements Callback<Map> {

	@Override
	public void callback(Map map) {

		// get transformation and scale
		ArUcoMap arucoMap = map.getMarkerMap();
		if (arucoMap == null) {
			Utils.pl("No marker map found. Skipping similarity transformation save.");
			return;
		}

		double scale = arucoMap.getScale();
		Transformation transformation = arucoMap.getTransformation();

		// build JSON and save to file
		JSONObject objJSON = new JSONObject();
		objJSON.put("scale", scale);

		JSONObject poseJSON = new JSONObject();
		poseJSON.put("Cx", transformation.getCx());
		poseJSON.put("Cy", transformation.getCy());
		poseJSON.put("Cz", transformation.getCz());
		poseJSON.put("qw", transformation.getQw());
		poseJSON.put("qx", transformation.getQx());
		poseJSON.put("qy", transformation.getQy());
		poseJSON.put("qz", transformation.getQz());
		objJSON.put("pose", poseJSON);

		printToFile(Parameters.<String>get("markerSimTransformPath"), objJSON.toString(2));
		Utils.pl("Saved marker similarity transform to " + Parameters.<String>get("markerSimTransformPath") + ".");

	}

	public static void printToFile(String filename, String output) {
		try {
			Files.write(Paths.get(filename), output.getBytes());
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

}
