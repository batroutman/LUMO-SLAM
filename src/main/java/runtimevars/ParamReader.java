package runtimevars;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.io.FileUtils;
import org.json.JSONArray;
import org.json.JSONObject;

import toolbox.Utils;

public class ParamReader {

	public void setParameters(String paramFile) {

		// read JSON from file
		String json = "";
		try {
			json = FileUtils.readFileToString(new File(paramFile), "utf-8");
		} catch (Exception e) {
			Utils.pl("Error reading JSON parameters. Falling back to default parameter values.");
			e.printStackTrace();
			return;
		}

		// set JSONObject
		JSONObject obj = new JSONObject(json);

		// set each parameter
		this.setString(obj, "inputDataPath");
		this.setString(obj, "bufferType");
		this.setString(obj, "bowVocabFile");

		this.setString(obj, "trajectorySavePath");
		this.setString(obj, "keyframeTrajectorySavePath");
		this.setString(obj, "movingObjectTrajectoryPathJSON");
		this.setString(obj, "movingObjectTrajectoryPathTXT");
		this.setString(obj, "markerSimTransformPath");

		this.setFloat(obj, "fx");
		this.setFloat(obj, "fy");
		this.setFloat(obj, "cx");
		this.setFloat(obj, "cy");
		this.setFloat(obj, "s");
		this.setListDouble(obj, "distCoeffs");

		this.setInteger(obj, "width");
		this.setInteger(obj, "height");
		this.setInteger(obj, "screenWidth");
		this.setInteger(obj, "screenHeight");

		this.setBoolean(obj, "smartInitialization");
		this.setDouble(obj, "parallaxRequirement");
		this.setDouble(obj, "initializationReprojectionError");

		this.setInteger(obj, "cellSize");
		this.setInteger(obj, "binMinCapacity");
		this.setInteger(obj, "binMaxCapacity");
		this.setInteger(obj, "maxFeatures");

		this.setDouble(obj, "clusterLength");

		this.setFloat(obj, "PnPRansacReprojectionThreshold");

		this.setInteger(obj, "maxRegistrationAttempts");
		this.setListString(obj, "movingObjectLabels");

		this.setDouble(obj, "triangulationBaseline");

		this.setInteger(obj, "LCMatchThreshold");
		this.setInteger(obj, "LCMatchMin");

		this.setListListFloat(obj, "cubes");

		this.setBoolean(obj, "usingMarkers");
		this.setString(obj, "markerMapFile");
		this.setListListInteger(obj, "movingObjectMarkers");

		this.setBoolean(obj, "ARView.showMapPoints");
		this.setBoolean(obj, "ARView.showBoundingBoxes");
		this.setBoolean(obj, "ARView.showObjectLabels");

		this.setString(obj, "GUI.view");
		this.setString(obj, "GUI.featureDisplayType");
		this.setString(obj, "GUI.mapColorScheme");

		this.setBoolean(obj, "MapView.followCamera");
		this.setListDouble(obj, "MapView.cameraOffset");

	}

	public void setString(JSONObject obj, String var) {

		try {
			String value = obj.getString(var);
			Parameters.<String>put(var, value);
		} catch (Exception e) {
			Utils.pl("Error parsing '" + var + "'. Falling back to default value.");
		}

	}

	public void setFloat(JSONObject obj, String var) {

		try {
			Float value = obj.getFloat(var);
			Parameters.<Float>put(var, value);
		} catch (Exception e) {
			Utils.pl("Error parsing '" + var + "'. Falling back to default value.");
		}

	}

	public void setDouble(JSONObject obj, String var) {

		try {
			Double value = obj.getDouble(var);
			Parameters.<Double>put(var, value);
		} catch (Exception e) {
			Utils.pl("Error parsing '" + var + "'. Falling back to default value.");
		}

	}

	public void setInteger(JSONObject obj, String var) {

		try {
			Integer value = obj.getInt(var);
			Parameters.<Integer>put(var, value);
		} catch (Exception e) {
			Utils.pl("Error parsing '" + var + "'. Falling back to default value.");
		}

	}

	public void setBoolean(JSONObject obj, String var) {

		try {
			Boolean value = obj.getBoolean(var);
			Parameters.<Boolean>put(var, value);
		} catch (Exception e) {
			Utils.pl("Error parsing '" + var + "'. Falling back to default value.");
		}

	}

	public void setListString(JSONObject obj, String var) {

		try {
			JSONArray jsonArray = obj.getJSONArray(var);
			List<String> listString = new ArrayList<String>();
			for (int i = 0; i < jsonArray.length(); i++) {
				listString.add(jsonArray.getString(i));
			}
			Parameters.<List<String>>put(var, listString);
		} catch (Exception e) {
			Utils.pl("Error parsing '" + var + "'. Falling back to default value.");
		}

	}

	public void setListDouble(JSONObject obj, String var) {

		try {
			JSONArray jsonArray = obj.getJSONArray(var);
			List<Double> listDouble = new ArrayList<Double>();
			for (int i = 0; i < jsonArray.length(); i++) {
				listDouble.add(jsonArray.getDouble(i));
			}
			Parameters.<List<Double>>put(var, listDouble);
		} catch (Exception e) {
			Utils.pl("Error parsing '" + var + "'. Falling back to default value.");
		}

	}

	public void setListListFloat(JSONObject obj, String var) {

		try {
			JSONArray jsonArray = obj.getJSONArray(var);
			List<List<Float>> listListFloat = new ArrayList<List<Float>>();
			for (int i = 0; i < jsonArray.length(); i++) {
				List<Float> listFloat = new ArrayList<Float>();
				for (int j = 0; j < jsonArray.getJSONArray(i).length(); j++) {
					listFloat.add(jsonArray.getJSONArray(i).getFloat(j));
				}
				listListFloat.add(listFloat);
			}
			Parameters.<List<List<Float>>>put(var, listListFloat);
		} catch (Exception e) {
			Utils.pl("Error parsing '" + var + "'. Falling back to default value.");
		}

	}

	public void setListListInteger(JSONObject obj, String var) {

		try {
			JSONArray jsonArray = obj.getJSONArray(var);
			List<List<Integer>> listListInteger = new ArrayList<List<Integer>>();
			for (int i = 0; i < jsonArray.length(); i++) {
				List<Integer> listInteger = new ArrayList<Integer>();
				for (int j = 0; j < jsonArray.getJSONArray(i).length(); j++) {
					listInteger.add(jsonArray.getJSONArray(i).getInt(j));
				}
				listListInteger.add(listInteger);
			}
			Parameters.<List<List<Integer>>>put(var, listListInteger);
		} catch (Exception e) {
			Utils.pl("Error parsing '" + var + "'. Falling back to default value.");
		}

	}

}
