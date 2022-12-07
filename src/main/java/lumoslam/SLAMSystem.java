package lumoslam;

import java.util.ArrayList;
import java.util.List;

import buffers.Buffer;
import types.Callback;
import types.FramePack;
import types.PipelineOutput;

// T - type that will be passed to callback functions when the system has completed (used to gather keyframes and record their trajectories)
public abstract class SLAMSystem<T> {

	protected Buffer<FramePack> inputBuffer = null;
	protected Buffer<PipelineOutput> outputBuffer = null;
	protected List<Callback<T>> whenFinishedCallbacks = new ArrayList<Callback<T>>();

	public SLAMSystem(Buffer<FramePack> inputBuffer, Buffer<PipelineOutput> outputBuffer) {
		this.inputBuffer = inputBuffer;
		this.outputBuffer = outputBuffer;
	}

	public abstract void start();

	public abstract void stop();

	public void whenFinished(Callback<T> cb) {
		this.whenFinishedCallbacks.add(cb);
	}

	protected void triggerFinished() {
		for (Callback<T> cb : whenFinishedCallbacks) {
			this.callFinishedCallback(cb);
		}
	}

	protected abstract void callFinishedCallback(Callback<T> cb);

	public Buffer<FramePack> getInputBuffer() {
		return inputBuffer;
	}

	public void setInputBuffer(Buffer<FramePack> inputBuffer) {
		this.inputBuffer = inputBuffer;
	}

	public Buffer<PipelineOutput> getOutputBuffer() {
		return outputBuffer;
	}

	public void setOutputBuffer(Buffer<PipelineOutput> outputBuffer) {
		this.outputBuffer = outputBuffer;
	}

	public List<Callback<T>> getWhenFinishedCallbacks() {
		return whenFinishedCallbacks;
	}

	public void setWhenFinishedCallbacks(List<Callback<T>> whenFinishedCallbacks) {
		this.whenFinishedCallbacks = whenFinishedCallbacks;
	}

}
