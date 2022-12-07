package buffers;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import types.FramePack;

public class MP4Buffer implements Buffer<FramePack> {

	public long frameLimit = 999999;
	protected List<FramePack> frames = new ArrayList<FramePack>();

	protected String filename = "";
	protected VideoCapture vc = new VideoCapture();

	private MP4Buffer() {

	}

	public MP4Buffer(String filename) {
		this.filename = filename;
		this.loadVideo();
	}

	protected void loadVideo() {
		if (!this.vc.isOpened())
			this.vc.open(filename);
		if (this.vc.isOpened()) {
			Mat mat = new Mat();
			for (int i = 0; i < this.frameLimit && vc.read(mat); i++) {
				if (i % 50 == 0) {
					System.out.println(i + " frames loaded...");
				}

				// create frame pack
				Mat rawFrame = mat.clone();
				Mat processedFrame = new Mat();
				Imgproc.cvtColor(rawFrame, rawFrame, Imgproc.COLOR_BGR2RGB);
				Imgproc.cvtColor(rawFrame, processedFrame, Imgproc.COLOR_RGB2GRAY);
				byte[] rawBuffer = new byte[rawFrame.rows() * rawFrame.cols() * 3];
				byte[] processedBuffer = new byte[processedFrame.rows() * processedFrame.cols()];
				rawFrame.get(0, 0, rawBuffer);
				processedFrame.get(0, 0, processedBuffer);
				FramePack framePack = new FramePack();
				framePack.setRawFrame(rawFrame);
				framePack.setProcessedFrame(processedFrame);
				framePack.setRawFrameBuffer(rawBuffer);
				framePack.setProcessedFrameBuffer(processedBuffer);

				// add to queue
				this.push(framePack);
			}
		}
	}

	@Override
	public void push(FramePack payload) {
		this.frames.add(payload);
	}

	@Override
	public FramePack getNext() {
		if (this.frames.size() > 0) {
			return this.frames.remove(0);
		} else {
			return null;
		}
	}

}
