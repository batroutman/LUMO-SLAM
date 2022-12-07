package datarecording;

import java.util.ArrayList;
import java.util.List;

import buffers.BufferListener;
import lumoslam.Map;
import toolbox.Utils;
import types.Callback;
import types.Dbl;
import types.PipelineOutput;

public class FrameRateStats implements Callback<Map>, BufferListener<PipelineOutput> {

	public List<Double> framerates = new ArrayList<Double>();

	@Override
	public void callback(Map map) {

		Dbl avg = new Dbl(0);
		Dbl median = new Dbl(0);
		Dbl stdDev = new Dbl(0);
		Utils.basicStats(this.framerates, avg, stdDev, median);

		Utils.pl("Framerate avg: " + avg.getValue());
		Utils.pl("Framerate median: " + median.getValue());
		Utils.pl("Framerate stdDev: " + stdDev.getValue());

	}

	@Override
	public void push(PipelineOutput payload) {
		this.framerates.add(payload.fps);
	}

}
