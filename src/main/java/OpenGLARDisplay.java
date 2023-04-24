
import static java.awt.Font.MONOSPACED;
import static org.lwjgl.glfw.GLFW.glfwTerminate;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.joml.Vector3f;
import org.liquidengine.legui.system.context.Context;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL30;
import org.opencv.core.KeyPoint;

import Jama.Matrix;
import arucomapping.ArUcoMap;
import buffers.Buffer;
import entities.Camera;
import entities.Entity;
import entities.LabelEntity;
import gui.GUIComponents;
import gui.InputHandler;
import lumoslam.Keyframe;
import lumoslam.Map;
import lumoslam.MapPoint;
import lumoslam.MovingModel;
import models.RawModel;
import models.TexturedModel;
import renderEngine.Loader;
import renderEngine.Renderer;
import runtimevars.Parameters;
import shaders.StaticShader;
import textures.Font;
import textures.ModelTexture;
import toolbox.Utils;
import types.Correspondence2D2D;
import types.PipelineOutput;
import types.Point3D;
import types.Pose;

public class OpenGLARDisplay {

	Buffer<PipelineOutput> pipelineBuffer = null;

//	Loader loader;
	Renderer renderer;
	Camera camera;
	Camera mapCamera;
	Pose mapTransformation = new Pose();
	ArrayList<Entity> entities = new ArrayList<Entity>();
	StaticShader cameraShader;
	Entity rawFrameEntity;
	Entity processedFrameEntity;
	StaticShader bgShader;
	StaticShader colorShader;
	HashMap<String, LabelEntity> labelEntities = new HashMap<String, LabelEntity>();

	// pipeline output
	Pose pose = new Pose();
	List<Correspondence2D2D> correspondences = new ArrayList<Correspondence2D2D>();
	List<Correspondence2D2D> prunedCorrespondences = new ArrayList<Correspondence2D2D>();
	List<Correspondence2D2D> untriangulatedCorrespondences = new ArrayList<Correspondence2D2D>();
	List<KeyPoint> trackedKeypoints = new ArrayList<KeyPoint>();
	List<KeyPoint> features = new ArrayList<KeyPoint>();
	List<Point3D> points = new ArrayList<Point3D>();
	List<Pose> poses = new ArrayList<Pose>();
	List<Keyframe> keyframes = new ArrayList<Keyframe>();
	Keyframe currentKeyframe = new Keyframe(0);
	Map map = new Map();
	HashMap<String, List<MapPoint>> mapPointsByPartition = new HashMap<String, List<MapPoint>>();
	List<MapPoint> trackedMapPoints = new ArrayList<MapPoint>();
	List<MovingModel> movingObjects = new ArrayList<MovingModel>();
	ArUcoMap markerMap = new ArUcoMap();

	List<Keyframe> closestKeyframes = new ArrayList<Keyframe>();
	List<Keyframe> loopDetectedKeyframes = new ArrayList<Keyframe>();

	// legui
	GUIComponents gui = new GUIComponents();

	// input handler
	InputHandler inputHandler = new InputHandler();

	public OpenGLARDisplay() {
		this.initOpenGL();
	}

	public OpenGLARDisplay(Buffer<PipelineOutput> pipelineBuffer) {
		this.pipelineBuffer = pipelineBuffer;
		this.initOpenGL();
	}

	public void initOpenGL() {

		List<Double> cameraOffset = Parameters.<List<Double>>get("MapView.cameraOffset");

		this.mapTransformation.setCx(cameraOffset.get(0));
		this.mapTransformation.setCy(cameraOffset.get(1));
		this.mapTransformation.setCz(cameraOffset.get(2));
		this.mapTransformation.rotateEuler(cameraOffset.get(3), cameraOffset.get(4), cameraOffset.get(5));

		// initialize
		this.gui.initGUI();
		this.inputHandler.setWindow(this.gui.window);

		this.cameraShader = new StaticShader(StaticShader.VERTEX_FILE, StaticShader.FRAGMENT_FILE);
		this.colorShader = new StaticShader(StaticShader.VERTEX_FILE, StaticShader.COLOR_FRAGMENT_FILE);
		StaticShader[] shaders = { this.cameraShader, this.colorShader };
		this.renderer = new Renderer(shaders);

		// temporarily set up cube data (eventually load from blender)
		float[] vertices = { -0.5f, 0.5f, 0, -0.5f, -0.5f, 0, 0.5f, -0.5f, 0, 0.5f, 0.5f, 0,

				-0.5f, 0.5f, 1, -0.5f, -0.5f, 1, 0.5f, -0.5f, 1, 0.5f, 0.5f, 1,

				0.5f, 0.5f, 0, 0.5f, -0.5f, 0, 0.5f, -0.5f, 1, 0.5f, 0.5f, 1,

				-0.5f, 0.5f, 0, -0.5f, -0.5f, 0, -0.5f, -0.5f, 1, -0.5f, 0.5f, 1,

				-0.5f, 0.5f, 1, -0.5f, 0.5f, 0, 0.5f, 0.5f, 0, 0.5f, 0.5f, 1,

				-0.5f, -0.5f, 1, -0.5f, -0.5f, 0, 0.5f, -0.5f, 0, 0.5f, -0.5f, 1

		};

		float[] textureCoords = { 0.25f, 0f, 0.25f, 0.25f, 0.5f, 0.25f, 0.5f, 0, 0.25f, 0.25f, 0.25f, 0.5f, 0.5f, 0.5f,
				0.5f, 0.25f, 0.25f, 0.5f, 0.25f, 0.75f, 0.5f, 0.75f, 0.5f, 0.5f, 0.25f, 0.75f, 0.25f, 1f, 0.5f, 1f,
				0.5f, 0.75f, 0f, 0.25f, 0f, 0.5f, 0.25f, 0.5f, 0.25f, 0.25f, 0.5f, 0.25f, 0.5f, 0.5f, 0.75f, 0.5f,
				0.75f, 0.25f };

		int[] indices = { 0, 1, 3, 3, 1, 2, 4, 5, 7, 7, 5, 6, 8, 9, 11, 11, 9, 10, 12, 13, 15, 15, 13, 14, 16, 17, 19,
				19, 17, 18, 20, 21, 23, 23, 21, 22

		};

		RawModel tModel = Loader.loadToVAO(vertices, textureCoords, indices);
		TexturedModel tStaticModel = new TexturedModel(tModel, new ModelTexture(Loader.loadTexture("solid_colors_64")));

		// add cubes
		List<List<Float>> cubes = Parameters.<List<List<Float>>>get("cubes");
		for (int i = 0; i < cubes.size(); i++) {
			List<Float> cube = cubes.get(i);
			this.entities.add(new Entity(tStaticModel, new Vector3f(cube.get(0), cube.get(1), cube.get(2)), cube.get(3),
					cube.get(4), cube.get(5), cube.get(6)));
		}

		// create labels
		this.labelEntities = this.getLabelEntityTable();

		// create camera
		this.camera = new Camera();
		this.mapCamera = new Camera();

		// create background data
		float[] bgVertices = { -1, 1, 0, -1, -1, 0, 1, -1, 0, 1, 1, 0 };

		int[] bgIndices = { 0, 1, 3, 3, 1, 2 };

		float[] bgTextureCoords = { 0, 0, 0, 1, 1, 1, 1, 0 };

		// set up background models
		this.bgShader = new StaticShader(StaticShader.BG_VERTEX_FILE, StaticShader.FRAGMENT_FILE);
		RawModel rawBgModel = Loader.loadToVAO(bgVertices, bgTextureCoords, bgIndices);
		TexturedModel rawBgStaticModel = new TexturedModel(rawBgModel,
				new ModelTexture(Loader.loadTexture("sample_texture")));
		this.rawFrameEntity = new Entity(rawBgStaticModel, new Vector3f(0, 0, -10), 0, 0, 0, 2000);

		RawModel processedBgModel = Loader.loadToVAO(bgVertices, bgTextureCoords, bgIndices);
		TexturedModel processedBgStaticModel = new TexturedModel(processedBgModel,
				new ModelTexture(Loader.loadTexture("sample_texture")));
		this.processedFrameEntity = new Entity(processedBgStaticModel, new Vector3f(0, 0, -10), 0, 0, 0, 2000);

	}

	public HashMap<String, LabelEntity> getLabelEntityTable() {

		Font fontAtlas = new Font(new java.awt.Font(MONOSPACED, java.awt.Font.BOLD, 128));
		HashMap<String, LabelEntity> labelEntities = new HashMap<String, LabelEntity>();
		List<String> labels = Parameters.<List<String>>get("movingObjectLabels");

		for (String label : labels) {
			labelEntities.put(label, null);
		}

		for (String label : labelEntities.keySet()) {
			LabelEntity labelEntity = new LabelEntity(label, fontAtlas, new Vector3f(-5, 0, 20), 0, 0, 0, 1.25f);
			labelEntities.put(label, labelEntity);
		}

		return labelEntities;

	}

	public void updateDisplay(Context context) {

		int screenWidth = Parameters.<Integer>get("screenWidth");
		int screenHeight = Parameters.<Integer>get("screenHeight");

		context.updateGlfwWindow();
		this.renderer.prepare();

		if (Parameters.<String>get("GUI.view").equalsIgnoreCase("AR")) {

			GL11.glViewport(0, 0, screenWidth, screenHeight);
			this.renderer.render(this.pose, this.camera, this.entities, this.labelEntities, this.cameraShader,
					this.colorShader, this.rawFrameEntity, this.bgShader, this.movingObjects,
					this.map.getAllMapPoints());

		} else if (Parameters.<String>get("GUI.view").equalsIgnoreCase("PROCESSED")) {

			GL11.glViewport(0, 0, screenWidth, screenHeight);
			this.renderer.renderProcessedView(this.processedFrameEntity, this.bgShader, this.prunedCorrespondences,
					this.untriangulatedCorrespondences, this.correspondences, this.features, this.trackedKeypoints);

		} else if (Parameters.<String>get("GUI.view").equalsIgnoreCase("MAP")) {

			GL11.glViewport(0, 0, screenWidth, screenHeight);
			synchronized (this.map) {
				this.renderer.renderMapView(this.mapCamera, this.colorShader, this.mapPointsByPartition,
						this.movingObjects, this.trackedMapPoints, this.keyframes, this.pose, this.currentKeyframe, 3,
						this.markerMap, this.closestKeyframes, this.loopDetectedKeyframes);
			}

		} else if (Parameters.<String>get("GUI.view").equalsIgnoreCase("ALL")) {

			synchronized (this.map) {
				GL11.glViewport(0, 0, screenWidth / 2, screenHeight / 2);
				this.renderer.render(this.pose, this.camera, this.entities, this.labelEntities, this.cameraShader,
						this.colorShader, this.rawFrameEntity, this.bgShader, this.movingObjects,
						this.map.getAllMapPoints());
				GL11.glViewport(screenWidth / 2, 0, screenWidth / 2, screenHeight / 2);
				this.renderer.renderProcessedView(this.processedFrameEntity, this.bgShader, this.prunedCorrespondences,
						this.untriangulatedCorrespondences, this.correspondences, this.features, this.trackedKeypoints);
				GL11.glViewport(screenWidth / 2, screenHeight / 2, screenWidth / 2, screenHeight / 2);
				this.renderer.renderMapView(this.mapCamera, this.colorShader, this.mapPointsByPartition,
						this.movingObjects, this.trackedMapPoints, this.keyframes, this.pose, this.currentKeyframe, 2,
						this.markerMap, this.closestKeyframes, this.loopDetectedKeyframes);
			}

		}

		// render gui frame
		this.gui.renderGUI();

	}

	public void setFrameToTexture(byte[] bytes, Entity bgEntity, boolean bgr) {

		ByteBuffer pixels = ByteBuffer.allocateDirect(bytes.length);
		pixels.put(bytes);
		pixels.flip();

		// delete old texture and create new texture
		GL11.glDeleteTextures(bgEntity.getModel().getTexture().getID());
		int textureID = GL11.glGenTextures();
		GL11.glBindTexture(GL11.GL_TEXTURE_2D, textureID);
		GL11.glPixelStorei(GL11.GL_UNPACK_ALIGNMENT, 1);
		GL11.glTexParameterf(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MIN_FILTER, GL11.GL_LINEAR);
		GL11.glTexParameterf(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MAG_FILTER, GL11.GL_LINEAR);
		if (bgr) {
			GL11.glTexImage2D(GL11.GL_TEXTURE_2D, 0, GL11.GL_RGB, Parameters.<Integer>get("width"),
					Parameters.<Integer>get("height"), 0, GL11.GL_RGB, GL11.GL_UNSIGNED_BYTE, pixels);
		} else {
			GL11.glTexImage2D(GL11.GL_TEXTURE_2D, 0, GL11.GL_LUMINANCE, Parameters.<Integer>get("width"),
					Parameters.<Integer>get("height"), 0, GL11.GL_LUMINANCE, GL11.GL_UNSIGNED_BYTE, pixels);
		}

		GL30.glGenerateMipmap(GL11.GL_TEXTURE_2D);

		// set bgEntity texture to new texture
		bgEntity.getModel().getTexture().setID(textureID);
	}

	public void setCameraPose(Pose pose) {
		this.camera.setMatrix(pose.getR00(), pose.getR01(), pose.getR02(), pose.getR10(), pose.getR11(), pose.getR12(),
				pose.getR20(), pose.getR21(), pose.getR22(), pose.getTx(), pose.getTy(), pose.getTz());

		Matrix mapPos;
		if (Parameters.<Boolean>get("MapView.followCamera")) {
			mapPos = this.mapTransformation.getHomogeneousMatrix().times(pose.getHomogeneousMatrix());
		} else {
			mapPos = this.mapTransformation.getHomogeneousMatrix();
		}

		this.mapCamera.setMatrix(mapPos.get(0, 0), mapPos.get(0, 1), mapPos.get(0, 2), mapPos.get(1, 0),
				mapPos.get(1, 1), mapPos.get(1, 2), mapPos.get(2, 0), mapPos.get(2, 1), mapPos.get(2, 2),
				mapPos.get(0, 3), mapPos.get(1, 3), mapPos.get(2, 3));
	}

	public void detectChanges() {

		PipelineOutput output;
		synchronized (this.pipelineBuffer) {
			output = this.pipelineBuffer.getNext();
		}

		if (output == null) {
			return;
		}

		this.setCameraPose(output.pose);

		if (output.rawFrameBuffer != null) {
			this.setFrameToTexture(output.rawFrameBuffer, this.rawFrameEntity, true);
		}

		if (output.processedFrameBuffer != null) {
			this.setFrameToTexture(output.processedFrameBuffer, this.processedFrameEntity, false);
		}

		this.pose = output.pose;
		this.map = output.map;
		this.prunedCorrespondences = output.prunedCorrespondences;
		this.correspondences = output.correspondences;
		this.untriangulatedCorrespondences = output.untriangulatedCorrespondences;
		this.trackedKeypoints = output.trackedKeypoints;
		this.features = output.features;
		this.points = output.points;
		this.poses = output.cameras;
		this.keyframes = output.keyframes;
		this.currentKeyframe = output.currentKeyframe;
		this.mapPointsByPartition = output.mapPointsByPartition;
		this.trackedMapPoints = output.trackedMapPoints;
		this.movingObjects = output.movingObjects;
		this.markerMap = output.markerMap;

		this.closestKeyframes = output.closestKeyframes;
		this.loopDetectedKeyframes = output.map.getLoopDetectedKeyframes();

		this.gui.updateFpsLabel(output.fps);

		this.gui.updateLabelText("trackingStatusLabel", output.trackingSuccessful ? "Successful" : "Lost");
		this.gui.updateLabelText("frameNumLabel", output.frameNum + "");
		this.gui.updateLabelText("numFeaturesLabel", output.numFeatures + "");
		this.gui.updateLabelText("numCorrespondencesLabel", output.prunedCorrespondences.size() + "");
		this.gui.updateLabelText("numPointsLabel", output.points.size() + "");
		this.gui.updateLabelText("numKeyframesLabel", output.numKeyframes + "");
		this.gui.updateLabelText("numClustersLabel", output.mapPointsByPartition.keySet().size() + "");
		this.gui.updateLabelText("numMovingLabel", output.movingObjects.size() + "");

	}

	public void displayLoop() {

		Context context = this.gui.getInitializer().getContext();

		while (this.gui.isRunning()) {
			this.detectChanges();
			this.inputHandler.moveMapCamera(this.mapTransformation);
			this.updateDisplay(context);
		}

		Utils.pl("destroying gui");
		this.gui.destroy();
		glfwTerminate();
		this.cameraShader.cleanUp();
		Loader.cleanUp();

	}

}
