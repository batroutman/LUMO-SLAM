package buffers;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import types.FramePack;

// Offline frame buffer designed to load in images from the TUM dataset
public class TUMBuffer implements Buffer<FramePack> {

	// DEBUG: limit the number of frames to pre-load
	public long frameLimit = 999999999;

	// preloaded frames
	ArrayList<FramePack> frames = new ArrayList<FramePack>();

	// timestamp and file location for each frame.
	// Ex. "1305031452.791720 rgb/1305031452.791720.png"
	ArrayList<String> frameData = new ArrayList<String>();

	// set to true to load all frames into the frame buffer before popping any
	boolean preloadFrames = true;

	// set to the directory that contains rgb.txt
	String filepath = "";

	private TUMBuffer() {

	}

	public TUMBuffer(String filepath, boolean preloadFrames) {
		this.filepath = filepath;
		this.preloadFrames = preloadFrames;

		// sanitize filepath
		if (filepath.charAt(filepath.length() - 1) != '/') {
			this.filepath = filepath + "/";
		}

		this.loadFramePaths();

		if (preloadFrames) {
			this.loadAllFrames();
		}

	}

	public void push(FramePack frame) {
		this.frames.add(frame);
	}

	public FramePack getNext() {
		if (this.preloadFrames) {
			return this.getPreloadedFrame();
		} else {
			return this.loadNextFrame();
		}
	}

	private FramePack getPreloadedFrame() {
		if (this.frames.size() > 0) {
			this.frameData.remove(0);
			return this.frames.remove(0);
		} else {
			return null;
		}
	}

	// load a single frame based on the current frame data
	private FramePack loadNextFrame() {
		if (this.frameData.size() > 0) {
			String data = this.frameData.remove(0);

			String splits[] = data.split(" ");
			long timestamp = (long) (Double.parseDouble(splits[0]) * 1000000);
			timestamp *= 1000;
			FramePack frame = this.loadFrame(this.filepath + splits[1]);
			frame.setTimestamp(timestamp);

			return frame;

		} else {
			return null;
		}
	}

	public void loadFramePaths() {
		System.out.println("Loading frame paths...");
		BufferedReader reader;
		try {
			reader = new BufferedReader(new FileReader(this.filepath + "rgb.txt"));
			String line = "";
			while (line != null && this.frameData.size() < this.frameLimit) {
				line = line.trim();
				if (line.length() != 0 && line.charAt(0) != '#') {

					this.frameData.add(line);

				}
				line = reader.readLine();
			}
		} catch (Exception e) {
			System.out.println("Problem loading file paths in TUMFrameBuffer::loadFilePaths()");
			e.printStackTrace();
		}
		System.out.println("Complete. Number of frames: " + this.frameData.size());
	}

	public void loadAllFrames() {

		System.out.println("Pre-loading frames...");

		for (int i = 0; i < this.frameData.size(); i++) {

			if (i % 100 == 0) {
				System.out.println(i + " frames loaded...");
			}

			// get the timestamp and title
			String splits[] = this.frameData.get(i).split(" ");
			long timestamp = (long) (Double.parseDouble(splits[0]) * 1000000);
			timestamp *= 1000;
			String title = splits[0];

			FramePack frame = this.loadFrame(this.filepath + splits[1]);

			frame.setTimestamp(timestamp);
			frame.setFrameTitle(title);

			this.frames.add(frame);

		}

		System.out.println("Complete. " + this.frames.size() + " frames loaded.");

	}

	public FramePack loadFrame(String fullPath) {

		Mat rawFrame = Imgcodecs.imread(fullPath);
		Mat processedFrame = new Mat();

		Imgproc.cvtColor(rawFrame, rawFrame, Imgproc.COLOR_BGR2RGB);
		Imgproc.cvtColor(rawFrame, processedFrame, Imgproc.COLOR_RGB2GRAY);

		byte[] rawBuffer = new byte[rawFrame.rows() * rawFrame.cols() * 3];
		byte[] processedBuffer = new byte[processedFrame.rows() * processedFrame.cols()];
		rawFrame.get(0, 0, rawBuffer);
		processedFrame.get(0, 0, processedBuffer);

//		Utils.pl("rawBuffer.length: " + rawBuffer.length);
//		Utils.pl("processedBuffer.length: " + processedBuffer.length);

		FramePack framePack = new FramePack();
		framePack.setRawFrame(rawFrame);
		framePack.setProcessedFrame(processedFrame);
		framePack.setRawFrameBuffer(rawBuffer);
		framePack.setProcessedFrameBuffer(processedBuffer);

		return framePack;

	}

}
