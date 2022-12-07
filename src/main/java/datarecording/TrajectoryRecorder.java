package datarecording;

import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import buffers.BufferListener;
import lumoslam.Keyframe;
import lumoslam.Map;
import runtimevars.Parameters;
import toolbox.Utils;
import types.Callback;
import types.FramePose;
import types.PipelineOutput;
import types.Pose;

public class TrajectoryRecorder implements Callback<Map>, BufferListener<PipelineOutput> {

	List<FramePose> SLAMOutput = new ArrayList<FramePose>();

	public void callback(Map map) {
		this.saveTrajectory(Parameters.<String>get("trajectorySavePath"));
		this.saveKeyframeTrajectory(map.getKeyframes(), Parameters.<String>get("keyframeTrajectorySavePath"));
	}

	public void push(PipelineOutput payload) {

		if (payload == null) {
			return;
		}

		FramePose framePose = new FramePose();
		framePose.setFrameTitle(payload.frameTitle);
		framePose.setPose(new Pose(payload.pose));
		this.SLAMOutput.add(framePose);

	}

	public String getTrajectoryText(List<FramePose> framePoses) {
		String text = "# estimated trajectory\n";
		String datePattern = "MM-dd-yyyy-HH:mm:ss";
		SimpleDateFormat sdf = new SimpleDateFormat(datePattern);
		text += "# time recorded: " + sdf.format(new Date()) + "\n";
		text += "# # timestamp tx ty tz qx qy qz qw\n";

		for (FramePose framePose : framePoses) {
			String line = "";
			line += framePose.getFrameTitle() + " ";
			line += framePose.getPose().getCx() + " ";
			line += framePose.getPose().getCy() + " ";
			line += framePose.getPose().getCz() + " ";
			line += framePose.getPose().getQx() + " ";
			line += framePose.getPose().getQy() + " ";
			line += framePose.getPose().getQz() + " ";
			line += framePose.getPose().getQw() + "\n";
			text += line;
		}

		return text;
	}

	public void saveTrajectory(String filename) {

		String trajectoryText = this.getTrajectoryText(this.SLAMOutput);

		this.writeTrajectoryTextToFile(trajectoryText, filename, "Trajectory");

	}

	public void saveKeyframeTrajectory(List<Keyframe> keyframes, String filename) {

		// generate list of frame poses from keyframes
		List<FramePose> kfFramePoses = new ArrayList<FramePose>();
		for (Keyframe kf : keyframes) {
			FramePose fp = new FramePose();
			fp.setFrameTitle(kf.getFrameTitle());
			fp.setPose(kf.getPose());
			kfFramePoses.add(fp);
		}

		// get trajectory text
		String trajectoryText = this.getTrajectoryText(kfFramePoses);

		// print to specified file
		this.writeTrajectoryTextToFile(trajectoryText, filename, "Keyframe trajectory");

	}

	public void writeTrajectoryTextToFile(String trajectoryText, String filename, String label) {
		PrintWriter out = null;
		try {

			out = new PrintWriter(filename);
			out.print(trajectoryText);

		} catch (Exception e) {
			e.printStackTrace();
		} finally {
			if (out != null) {
				out.close();
			}
		}

		Utils.pl(label + " saved to " + filename + ".");
	}

}
