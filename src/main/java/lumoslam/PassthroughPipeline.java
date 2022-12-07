package lumoslam;

import java.util.List;
import java.util.Random;

import Jama.Matrix;
import buffers.Buffer;
import buffers.QueuedBuffer;
import buffers.SingletonBuffer;
import runtimevars.Parameters;
import toolbox.Utils;
import types.Callback;
import types.Correspondence2D2D;
import types.FramePack;
import types.PipelineOutput;
import types.Point3D;
import types.Pose;

public class PassthroughPipeline extends SLAMSystem<List<Keyframe>> {

	int frameNum = 0;
	double tz = 0;
	double t = 0;
	double qw = 0.707;
	double rotY = 0;
	Matrix rotation = new Matrix(4, 1);
	Matrix rotChange = new Matrix(4, 1);

	protected void callFinishedCallback(Callback<List<Keyframe>> cb) {
		cb.callback(null);
	}

	Thread poseEstimationThread = new Thread() {
		@Override
		public void run() {
			mainloop();
		}
	};

	public PassthroughPipeline() {
		super(new QueuedBuffer<FramePack>(), new SingletonBuffer<PipelineOutput>());
	}

	public PassthroughPipeline(Buffer<FramePack> inputBuffer, Buffer<PipelineOutput> outputBuffer) {
		super(inputBuffer, outputBuffer);
		rotation.set(0, 0, 1);

		// rotate x
//		rotChange.set(0, 0, 1);
//		rotChange.set(1, 0, 0.009);

		// rotate y
//		rotChange.set(0, 0, 1);
//		rotChange.set(2, 0, 0.009);
//
		// rotate z
		rotChange.set(0, 0, 1);
		rotChange.set(3, 0, 0.009);
		rotChange = rotChange.times(1 / rotChange.normF());
	}

	@Override
	public void start() {
		this.poseEstimationThread.start();
	}

	@Override
	public void stop() {
		this.poseEstimationThread.interrupt();
	}

	public void mainloop() {
		boolean keepGoing = true;
		while (keepGoing) {
			long start = System.currentTimeMillis();
			FramePack newFrame = inputBuffer.getNext();
			if (newFrame == null) {
				keepGoing = false;
				continue;
			}

			PipelineOutput po = new PipelineOutput();
			po.frameNum = this.frameNum++;
			po.rawFrame = newFrame.getRawFrame();
			po.processedFrame = newFrame.getProcessedFrame();
			po.rawFrameBuffer = newFrame.getRawFrameBuffer();
			po.processedFrame.get(0, 0, newFrame.getProcessedFrameBuffer());
			po.processedFrameBuffer = newFrame.getProcessedFrameBuffer();

			Random rand = new Random(100);

			for (int i = 0; i < 100; i++) {
				double x0 = rand.nextDouble() * Parameters.<Integer>get("width");
				double y0 = rand.nextDouble() * Parameters.<Integer>get("height");
				double x1 = rand.nextDouble() * Parameters.<Integer>get("width");
				double y1 = rand.nextDouble() * Parameters.<Integer>get("height");
				po.correspondences.add(new Correspondence2D2D(x0, y0, x1, y1));
//				po.features.add(new Feature(x1, y1, (int) (15 * rand.nextDouble()) + 5));
			}

			// map points
			po.points.add(new Point3D(1, 1, 10));
			for (int i = 0; i < 1000; i++) {
				double x = rand.nextDouble() * 2 - 1;
				double y = rand.nextDouble() * 2 - 1;
				double z = rand.nextDouble() * 4 - 1;
				po.points.add(new Point3D(x, y, z));
			}

			// pose
			Pose pose = new Pose();
			pose.setQw(rotation.get(0, 0));
			pose.setQx(rotation.get(1, 0));
			pose.setQy(rotation.get(2, 0));
			pose.setQz(rotation.get(3, 0));
//			pose.setCz(t);
//			t -= 0.01;
			po.pose = pose;
			rotation = Utils.quatMult(rotChange, rotation);

			try {
				Thread.sleep(100);
			} catch (Exception e) {
			}

			long end = System.currentTimeMillis();
			po.fps = 1000 / (end - start + 1);

			outputBuffer.push(po);

		}
	}

}
