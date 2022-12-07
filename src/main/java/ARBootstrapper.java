import java.util.Arrays;
import java.util.List;

import org.opencv.core.Core;

import buffers.Buffer;
import buffers.MP4Buffer;
import buffers.SingletonBuffer;
import buffers.TUMBuffer;
import datarecording.FrameRateStats;
import datarecording.MarkerSimTransformRecorder;
import datarecording.MovingObjectTrajectoryRecorder;
import datarecording.TrajectoryRecorder;
import lumoslam.Map;
import lumoslam.LUMOSLAM;
import lumoslam.SLAMSystem;
import placerecognition.Vocabulary;
import runtimevars.ParamReader;
import runtimevars.Parameters;
import types.FramePack;
import types.PipelineOutput;

public class ARBootstrapper {

	public ARBootstrapper() {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	public void start() {

		// set up input and output buffers
		SingletonBuffer<PipelineOutput> outputBuffer = new SingletonBuffer<PipelineOutput>();
		Buffer<FramePack> inputBuffer;
		TrajectoryRecorder trajectoryRecorder = new TrajectoryRecorder();
		outputBuffer.addListener(trajectoryRecorder);
		FrameRateStats frameRateStats = new FrameRateStats();
		outputBuffer.addListener(frameRateStats);
		MarkerSimTransformRecorder mstRecorder = new MarkerSimTransformRecorder();
		MovingObjectTrajectoryRecorder motRecorder = new MovingObjectTrajectoryRecorder();
		outputBuffer.addListener(motRecorder);

		String bufferType = Parameters.<String>get("bufferType");
		String filepath = Parameters.<String>get("inputDataPath");

		Vocabulary.global.loadBoW(Parameters.<String>get("bowVocabFile"));

		if (bufferType.equals("TUM")) {
			inputBuffer = new TUMBuffer(filepath, true);
		} else {
			inputBuffer = new MP4Buffer(filepath);
		}

		// instantiate the SLAM system with the buffers
		SLAMSystem<Map> pipeline = new LUMOSLAM(inputBuffer, outputBuffer);

		// record trajectory and keyframe trajectory to file when system is done
		pipeline.whenFinished(trajectoryRecorder);

		// get framerate statistics
		pipeline.whenFinished(frameRateStats);

		// if there is a marker map, print the transformation offset info
		pipeline.whenFinished(mstRecorder);

		// record moving object data
		pipeline.whenFinished(motRecorder);

		// instantiate the display
		OpenGLARDisplay ARDisplay = new OpenGLARDisplay(outputBuffer);

		// bootstrap the system and the display
		pipeline.start();
		ARDisplay.displayLoop();

		println("Done.");
	}

	public static void println(Object obj) {
		System.out.println(obj);
	}

	public void handleArgs(String[] args) throws Exception {

		List<String> listArgs = Arrays.asList(args);

		if (listArgs.indexOf("-config") != -1) {

			if (listArgs.size() < listArgs.indexOf("-config")) {
				throw new Exception("-config argument must be followed by path to config file.");
			}

			ParamReader paramReader = new ParamReader();
			paramReader.setParameters(listArgs.get(listArgs.indexOf("-config") + 1));
		}

	}

	public static void main(String[] args) throws Exception {

		ARBootstrapper arBootstrapper = new ARBootstrapper();

		Parameters.setDefaultParameters();
		arBootstrapper.handleArgs(args);
		Parameters.printParams();

		arBootstrapper.start();

	}

}