package datarecording;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

import org.json.JSONArray;
import org.json.JSONObject;

import buffers.BufferListener;
import lumoslam.Map;
import lumoslam.MovingModel;
import runtimevars.Parameters;
import types.Callback;
import types.PipelineOutput;
import types.Transformation;

public class MovingObjectTrajectoryRecorder implements Callback<Map>, BufferListener<PipelineOutput> {

	public static class MOSnapshot {
		public double frameNum;
		public String frameTitle;
		public List<MovingModel> movingObjects;

		public MOSnapshot(double frameNum, String frameTitle, List<MovingModel> movingObjects) {
			this.frameNum = frameNum;
			this.frameTitle = frameTitle;
			this.movingObjects = movingObjects;
		}
	}

	public List<MOSnapshot> moSnapshots = new ArrayList<MOSnapshot>();

	@Override
	public void callback(Map map) {
		this.saveJSON();
		this.saveTXT();
	}

	// saves the first moving object trajectory (when it exists)
	public void saveTXT() {

		// get the different MO labels
		HashMap<String, Boolean> labels = new HashMap<String, Boolean>();
		for (MOSnapshot snapshot : this.moSnapshots) {
			for (MovingModel mo : snapshot.movingObjects) {
				labels.put(mo.getLabel(), true);
			}
		}

		// save trajectories for each moving object in different files
		for (String label : labels.keySet()) {

			String text = "";

			for (MOSnapshot snapshot : this.moSnapshots) {

				if (snapshot.movingObjects.size() == 0) {
					continue;
				}

				List<MovingModel> mos = snapshot.movingObjects.stream().filter(o -> o.getLabel().equals(label))
						.collect(Collectors.toList());
				if (mos == null || mos.size() == 0) {
					continue;
				}

				MovingModel mo = mos.get(0);
				Transformation t = mo.getTransformation();

				String line = "";
				line += snapshot.frameTitle + " ";
				line += t.getCx() + " ";
				line += t.getCy() + " ";
				line += t.getCz() + " ";
				line += t.getQx() + " ";
				line += t.getQy() + " ";
				line += t.getQz() + " ";
				line += t.getQw() + "\n";
				text += line;

			}

			printToFile(Parameters.<String>get("movingObjectTrajectoryPathTXT") + "-" + label + ".txt", text);

		}

	}

	public void saveJSON() {

		// build JSON and save to file
		JSONArray array = new JSONArray();

		for (MOSnapshot snapshot : this.moSnapshots) {

			JSONObject snapshotJSON = new JSONObject();
			snapshotJSON.put("frameNum", snapshot.frameNum);
			snapshotJSON.put("frameTitle", snapshot.frameTitle);

			JSONArray movingObjects = new JSONArray();
			for (MovingModel mo : snapshot.movingObjects) {
				JSONObject moJSON = new JSONObject();
				moJSON.put("id", mo.getLabel());

				JSONObject poseJSON = new JSONObject();
				poseJSON.put("Cx", mo.getTransformation().getCx());
				poseJSON.put("Cy", mo.getTransformation().getCy());
				poseJSON.put("Cz", mo.getTransformation().getCz());
				poseJSON.put("qw", mo.getTransformation().getQw());
				poseJSON.put("qx", mo.getTransformation().getQx());
				poseJSON.put("qy", mo.getTransformation().getQy());
				poseJSON.put("qz", mo.getTransformation().getQz());
				moJSON.put("pose", poseJSON);

				movingObjects.put(moJSON);
			}

			snapshotJSON.put("movingObjects", movingObjects);
			array.put(snapshotJSON);

		}

		printToFile(Parameters.<String>get("movingObjectTrajectoryPathJSON"), array.toString(2));

	}

	@Override
	public void push(PipelineOutput payload) {

		// make copies of moving objects
		List<MovingModel> movingObjects = new ArrayList<MovingModel>();
		for (MovingModel mo : payload.movingObjects) {
			movingObjects.add(new MovingModel(mo));
		}

		this.moSnapshots.add(new MOSnapshot(payload.frameNum, payload.frameTitle, movingObjects));
	}

	public static void printToFile(String filename, String output) {
		try {
			Files.write(Paths.get(filename), output.getBytes());
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

}
