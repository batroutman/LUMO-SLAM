package types;

public class FramePose {

	protected String frameTitle = "";
	protected Pose pose = new Pose();

	public FramePose() {

	}

	public String getFrameTitle() {
		return frameTitle;
	}

	public void setFrameTitle(String frameTitle) {
		this.frameTitle = frameTitle;
	}

	public Pose getPose() {
		return pose;
	}

	public void setPose(Pose pose) {
		this.pose = pose;
	}

}
