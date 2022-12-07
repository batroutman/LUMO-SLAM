package types;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

// bundles a raw frame and its processed frame (the raw frame that has undergone some degree of preprocessing for tracking)
public class FramePack {

	private long timestamp = 0;
	private String frameTitle = "";
	private Mat rawFrame = null;
	private Mat processedFrame = null;
	private byte[] rawFrameBuffer = null;
	private byte[] processedFrameBuffer = null;

	public void undistrortRawAndProcessedFrames(Mat K, Mat distCoeffs) {
		Mat newRaw = new Mat();
		Mat newProcessed = new Mat();
		Imgproc.undistort(this.rawFrame, newRaw, K, distCoeffs);
		Imgproc.undistort(this.processedFrame, newProcessed, K, distCoeffs);
		this.rawFrame = newRaw;
		this.processedFrame = newProcessed;
		this.updateRawBuffer();
		this.updateProcessedBuffer();
	}

	public void updateRawBuffer() {
		byte[] rawBuffer = new byte[rawFrame.rows() * rawFrame.cols() * 3];
		this.rawFrame.get(0, 0, rawBuffer);
		this.rawFrameBuffer = rawBuffer;
	}

	public void updateProcessedBuffer() {
		byte[] processedBuffer = new byte[processedFrame.rows() * processedFrame.cols() * 3];
		this.processedFrame.get(0, 0, processedBuffer);
		this.processedFrameBuffer = processedBuffer;
	}

	public long getTimestamp() {
		return timestamp;
	}

	public void setTimestamp(long timestamp) {
		this.timestamp = timestamp;
	}

	public Mat getRawFrame() {
		return rawFrame;
	}

	public void setRawFrame(Mat rawFrame) {
		this.rawFrame = rawFrame;
	}

	public Mat getProcessedFrame() {
		return processedFrame;
	}

	public void setProcessedFrame(Mat processedFrame) {
		this.processedFrame = processedFrame;
	}

	public byte[] getRawFrameBuffer() {
		return rawFrameBuffer;
	}

	public void setRawFrameBuffer(byte[] rawFrameBuffer) {
		this.rawFrameBuffer = rawFrameBuffer;
	}

	public byte[] getProcessedFrameBuffer() {
		return processedFrameBuffer;
	}

	public void setProcessedFrameBuffer(byte[] processedFrameBuffer) {
		this.processedFrameBuffer = processedFrameBuffer;
	}

	public String getFrameTitle() {
		return frameTitle;
	}

	public void setFrameTitle(String frameTitle) {
		this.frameTitle = frameTitle;
	}

}
