package renderEngine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL13;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GL30;
import org.opencv.core.KeyPoint;

import Jama.Matrix;
import arucomapping.ArUcoMap;
import entities.Camera;
import entities.Entity;
import entities.LabelEntity;
import gui.GUIComponents;
import lumoslam.Keyframe;
import lumoslam.MapPoint;
import lumoslam.MovingModel;
import models.RawModel;
import models.TexturedModel;
import runtimevars.Parameters;
import shaders.StaticShader;
import toolbox.Maths;
import toolbox.Utils;
import types.Correspondence2D2D;
import types.Point3D;
import types.Pose;
import types.Transformation;

public class Renderer {

	private static final float FOV = 70;
	private static final float NEAR_PLANE = 0.01f;
	private static final float FAR_PLANE = 50000;

	private static final Float[] blue = { 0f, 0f, 1f };
	private static final Float[] red = { 1f, 0f, 0f };

	private static HashMap<String, Float[]> partitionColors = new HashMap<String, Float[]>();

	private static Random random = new Random(0);

	private Matrix4f projectionMatrix;

	public Renderer(StaticShader[] perspectiveShaders) {
		createProjectionMatrix();
		for (StaticShader shader : perspectiveShaders) {
			shader.start();
			shader.loadProjectionMatrix(projectionMatrix);
			shader.stop();
		}
	}

	public void prepare() {

		GL11.glClearColor(1, 1, 1, 1);
		GL11.glClear(GL11.GL_COLOR_BUFFER_BIT | GL11.GL_DEPTH_BUFFER_BIT);

	}

	public float[] getHashedColor(Object o) {

		double hash = o.hashCode();
		double base = 2179;
		double r = hash * base % 256;
		double g = r * base % 256;
		double b = g * base % 256;

		r /= 255;
		g /= 255;
		b /= 255;

		float[] rgb = { (float) r, (float) g, (float) b };
//		Utils.pl("r: " + r + ", g: " + g + ", b: " + b);
		return rgb;

	}

	public void render(Pose pose, Camera camera, ArrayList<Entity> entities, HashMap<String, LabelEntity> labelEntities,
			StaticShader cameraShader, StaticShader colorShader, Entity background, StaticShader bgShader,
			List<MovingModel> movingObjects, List<MapPoint> mapPoints) {

		// background
		GL11.glDisable(GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc(GL11.GL_NEVER);
		bgShader.start();
		TexturedModel bgModel = background.getModel();
		RawModel bgRawModel = bgModel.getRawModel();
		GL30.glBindVertexArray(bgRawModel.getVaoID());
		GL20.glEnableVertexAttribArray(0);
		GL20.glEnableVertexAttribArray(1);
		GL13.glActiveTexture(GL13.GL_TEXTURE0);
		GL11.glBindTexture(GL11.GL_TEXTURE_2D, bgModel.getTexture().getID());
		GL11.glDrawElements(GL11.GL_TRIANGLES, bgRawModel.getVertexCount(), GL11.GL_UNSIGNED_INT, 0);
		GL20.glDisableVertexAttribArray(0);
		GL20.glDisableVertexAttribArray(1);
		GL30.glBindVertexArray(0);
		bgShader.stop();

		if (Parameters.<Boolean>get("ARView.showMapPoints")) {
			this.renderMapPoints(camera, colorShader, mapPoints, 5);
		}

		// bounding boxes (moving objects)
		for (MovingModel mo : movingObjects) {
			MovingModel.BoundingBox bb = mo.getBoundingBox();
			LabelEntity labelEntity = labelEntities.get(mo.getLabel());
			this.renderLabelEntity(pose, camera, cameraShader, mo, labelEntity);
			if (Parameters.<Boolean>get("ARView.showBoundingBoxes")) {
				this.renderBoundingBox(camera, colorShader, bb.x0, bb.x1, bb.y0, bb.y1, bb.z0, bb.z1, 1, 1, 1, 1);
			}
		}

		// 3D boxes
		GL11.glEnable(GL11.GL_DEPTH_TEST);
		GL11.glEnable(GL11.GL_BLEND);
		GL11.glBlendFunc(GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA);
		GL11.glDepthFunc(GL11.GL_LEQUAL);
		cameraShader.start();
		cameraShader.loadViewMatrix(camera);
		for (Entity entity : entities) {
			TexturedModel model = entity.getModel();
			RawModel rawModel = model.getRawModel();
			GL30.glBindVertexArray(rawModel.getVaoID());
			GL20.glEnableVertexAttribArray(0);
			GL20.glEnableVertexAttribArray(1);
			Matrix4f transformationMatrix = Maths.createRotatedTransformationMatrix(entity.getPosition(),
					entity.getRotX(), entity.getRotY(), entity.getRotZ(), entity.getScale());
			cameraShader.loadTransformationMatrix(transformationMatrix);
			GL13.glActiveTexture(GL13.GL_TEXTURE0);
			GL11.glBindTexture(GL11.GL_TEXTURE_2D, model.getTexture().getID());
			GL11.glDrawElements(GL11.GL_TRIANGLES, rawModel.getVertexCount(), GL11.GL_UNSIGNED_INT, 0);
			GL20.glDisableVertexAttribArray(0);
			GL20.glDisableVertexAttribArray(1);
			GL30.glBindVertexArray(0);
		}
		cameraShader.stop();

	}

	public void renderLabelEntity(Pose pose, Camera camera, StaticShader cameraShader, MovingModel movingObject,
			LabelEntity labelEntity) {

		if (labelEntity == null) {
			return;
		}

		GL11.glEnable(GL11.GL_DEPTH_TEST);
		GL11.glEnable(GL11.GL_BLEND);
		GL11.glBlendFunc(GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA);
		GL11.glDepthFunc(GL11.GL_LEQUAL);
		cameraShader.start();
		cameraShader.loadViewMatrix(camera);

		TexturedModel model = labelEntity.getModel();
		RawModel rawModel = model.getRawModel();
		GL30.glBindVertexArray(rawModel.getVaoID());
		GL20.glEnableVertexAttribArray(0);
		GL20.glEnableVertexAttribArray(1);

		// for the label position, pick a point on the bounding box
		Point3D centroid = movingObject.getTransformedCentroid();
		if (centroid == null) {
			centroid = new Point3D(0, 0, 0);
		}
		Vector3f position = new Vector3f((float) centroid.getX(), (float) centroid.getY(), (float) centroid.getZ());

		Matrix4f transformationMatrix = Maths.createRotatedTransformationMatrix(position, (float) pose.getRotXDeg(),
				(float) pose.getRotYDeg(), (float) pose.getRotZDeg(), labelEntity.getScale());
		cameraShader.loadTransformationMatrix(transformationMatrix);
		GL13.glActiveTexture(GL13.GL_TEXTURE0);
		GL11.glBindTexture(GL11.GL_TEXTURE_2D, model.getTexture().getID());
		GL11.glDrawElements(GL11.GL_TRIANGLES, rawModel.getVertexCount(), GL11.GL_UNSIGNED_INT, 0);
		GL20.glDisableVertexAttribArray(0);
		GL20.glDisableVertexAttribArray(1);
		GL30.glBindVertexArray(0);

		cameraShader.stop();
	}

	public void renderBoundingBox(Camera camera, StaticShader colorShader, double x0, double x1, double y0, double y1,
			double z0, double z1, float r, float g, float b, float thickness) {

		Vector3f range = new Vector3f((float) (x1 - x0), (float) (y1 - y0), (float) (z1 - z0));
		Vector3f range2 = new Vector3f(range.x() / 2, range.y() / 2, range.z() / 2);
		Vector3f center = new Vector3f((float) (x0 + range2.x()), (float) (y0 + range2.y()), (float) (z0 + range2.z()));

		GL11.glLineWidth(thickness);
		GL11.glEnable(GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc(GL11.GL_LEQUAL);
		colorShader.start();
		colorShader.loadViewMatrix(camera);
		colorShader.loadCustomColor(new Vector3f(r, g, b));
		Matrix4f transformationMatrix = Maths.createTransformationMatrix(center, 0, 0, 0, 1f);
		colorShader.loadTransformationMatrix(transformationMatrix);

		GL11.glBegin(GL11.GL_LINES);
		GL11.glVertex3d(-range2.x(), -range2.y(), -range2.z());
		GL11.glVertex3d(range2.x(), -range2.y(), -range2.z());

		GL11.glVertex3d(-range2.x(), -range2.y(), -range2.z());
		GL11.glVertex3d(-range2.x(), range2.y(), -range2.z());

		GL11.glVertex3d(-range2.x(), -range2.y(), -range2.z());
		GL11.glVertex3d(-range2.x(), -range2.y(), range2.z());

		GL11.glVertex3d(range2.x(), range2.y(), -range2.z());
		GL11.glVertex3d(-range2.x(), range2.y(), -range2.z());

		GL11.glVertex3d(range2.x(), range2.y(), -range2.z());
		GL11.glVertex3d(range2.x(), -range2.y(), -range2.z());

		GL11.glVertex3d(range2.x(), range2.y(), -range2.z());
		GL11.glVertex3d(range2.x(), range2.y(), range2.z());

		GL11.glVertex3d(range2.x(), -range2.y(), range2.z());
		GL11.glVertex3d(range2.x(), -range2.y(), -range2.z());

		GL11.glVertex3d(range2.x(), -range2.y(), range2.z());
		GL11.glVertex3d(range2.x(), range2.y(), range2.z());

		GL11.glVertex3d(range2.x(), -range2.y(), range2.z());
		GL11.glVertex3d(-range2.x(), -range2.y(), range2.z());

		GL11.glVertex3d(-range2.x(), range2.y(), range2.z());
		GL11.glVertex3d(range2.x(), range2.y(), range2.z());

		GL11.glVertex3d(-range2.x(), range2.y(), range2.z());
		GL11.glVertex3d(-range2.x(), -range2.y(), range2.z());

		GL11.glVertex3d(-range2.x(), range2.y(), range2.z());
		GL11.glVertex3d(-range2.x(), range2.y(), -range2.z());
		GL11.glEnd();

		colorShader.stop();

	}

	public void renderProcessedView(Entity background, StaticShader bgShader, List<Correspondence2D2D> correspondences,
			List<Correspondence2D2D> untriangulatedCorrespondences, List<Correspondence2D2D> initialCorrespondences,
			List<KeyPoint> features, List<KeyPoint> trackedKeypoints, GUIComponents.FEATURE_DISPLAY displayType) {

		GL11.glDisable(GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc(GL11.GL_NEVER);
		bgShader.start();
		TexturedModel bgModel = background.getModel();
		RawModel bgRawModel = bgModel.getRawModel();
		GL30.glBindVertexArray(bgRawModel.getVaoID());
		GL20.glEnableVertexAttribArray(0);
		GL20.glEnableVertexAttribArray(1);
		GL13.glActiveTexture(GL13.GL_TEXTURE0);
		GL11.glBindTexture(GL11.GL_TEXTURE_2D, bgModel.getTexture().getID());
		GL11.glDrawElements(GL11.GL_TRIANGLES, bgRawModel.getVertexCount(), GL11.GL_UNSIGNED_INT, 0);
		GL20.glDisableVertexAttribArray(0);
		GL20.glDisableVertexAttribArray(1);
		GL30.glBindVertexArray(0);
		bgShader.stop();
		if (displayType == GUIComponents.FEATURE_DISPLAY.BOXES) {
			this.renderFeatureBoxes(features);
		} else if (displayType == GUIComponents.FEATURE_DISPLAY.POINTS) {
			this.renderFeaturePoints(features);
		} else if (displayType == GUIComponents.FEATURE_DISPLAY.TRACKED_POINTS) {
			this.renderFeaturePoints(trackedKeypoints);
		}

		this.renderCorrespondences(initialCorrespondences, 1, 0, 0);
		this.renderCorrespondences(untriangulatedCorrespondences, 1, 0.5f, 0);
		this.renderCorrespondences(correspondences);
		this.renderBinLines(Parameters.<Integer>get("cellSize"), Parameters.<Integer>get("cellSize"), 0, 0.5f, 0);

	}

	public void renderBinLines(int binWidth, int binHeight, float r, float g, float b) {

		double width = Parameters.<Integer>get("width").doubleValue();
		double height = Parameters.<Integer>get("height").doubleValue();

		GL11.glColor3f(r, g, b);
		GL11.glLineWidth(1.0f);
		GL11.glMatrixMode(GL11.GL_PROJECTION);
		GL11.glLoadIdentity();
		GL11.glOrtho(0, width, height, 0, 0, 100);

		// vertical lines
		for (int xPos = 0; xPos < width; xPos += binWidth) {
			GL11.glBegin(GL11.GL_LINES);
			GL11.glVertex2d(xPos, 0);
			GL11.glVertex2d(xPos, height);
			GL11.glEnd();
		}

		// horizontal lines
		for (int yPos = 0; yPos < height; yPos += binHeight) {
			GL11.glBegin(GL11.GL_LINES);
			GL11.glVertex2d(0, yPos);
			GL11.glVertex2d(width, yPos);
			GL11.glEnd();
		}
	}

	public void renderCorrespondences(List<Correspondence2D2D> correspondences) {
		// default to green
		this.renderCorrespondences(correspondences, 0, 1, 0);
	}

	public void renderCorrespondences(List<Correspondence2D2D> correspondences, float r, float g, float b) {

		double width = Parameters.<Integer>get("width").doubleValue();
		double height = Parameters.<Integer>get("height").doubleValue();

		GL11.glColor3f(r, g, b);
		GL11.glLineWidth(2.0f);
		GL11.glMatrixMode(GL11.GL_PROJECTION);
		GL11.glLoadIdentity();
		GL11.glOrtho(0, width, height, 0, 0, 100);
		for (Correspondence2D2D corr : correspondences) {
			GL11.glBegin(GL11.GL_LINES);
			GL11.glVertex2d(corr.getX0(), corr.getY0());
			GL11.glVertex2d(corr.getX1(), corr.getY1());
			GL11.glEnd();
		}
	}

	public void renderFeaturePoints(List<KeyPoint> features) {

		double width = Parameters.<Integer>get("width").doubleValue();
		double height = Parameters.<Integer>get("height").doubleValue();

		GL11.glColor3f(0, 0, 1);
		GL11.glPointSize(5.0f);
		GL11.glMatrixMode(GL11.GL_PROJECTION);
		GL11.glLoadIdentity();
		GL11.glOrtho(0, width, height, 0, 0, 100);
		GL11.glBegin(GL11.GL_POINTS);
		for (KeyPoint feature : features) {
			GL11.glVertex2d(feature.pt.x, feature.pt.y);
		}
		GL11.glEnd();
	}

	public void renderFeatureBoxes(List<KeyPoint> features) {

		double width = Parameters.<Integer>get("width").doubleValue();
		double height = Parameters.<Integer>get("height").doubleValue();

		GL11.glColor3f(0, 0, 1);
		GL11.glLineWidth(2.0f);
		GL11.glMatrixMode(GL11.GL_PROJECTION);
		GL11.glLoadIdentity();
		GL11.glOrtho(0, width, height, 0, 0, 100);
		for (KeyPoint feature : features) {
			double topLeftX = feature.pt.x - (feature.size / 2) - 1;
			double topLeftY = feature.pt.y - (feature.size / 2) - 1;
			double patchSize = feature.size + 2;
			GL11.glBegin(GL11.GL_LINES);
			GL11.glVertex2d(topLeftX, topLeftY);
			GL11.glVertex2d(topLeftX + patchSize, topLeftY);
			GL11.glVertex2d(topLeftX + patchSize, topLeftY);
			GL11.glVertex2d(topLeftX + patchSize, topLeftY + patchSize);
			GL11.glVertex2d(topLeftX + patchSize, topLeftY + patchSize);
			GL11.glVertex2d(topLeftX, topLeftY + patchSize);
			GL11.glVertex2d(topLeftX, topLeftY + patchSize);
			GL11.glVertex2d(topLeftX, topLeftY);
			GL11.glEnd();
		}
	}

	// old
	public void renderMapView(Camera mapCamera, StaticShader cameraShader, List<Point3D> mapPoints,
			List<Keyframe> keyframes, Pose mainPose, Keyframe currentKeyframe, float pointSize) {

		this.renderPoints(mapCamera, cameraShader, mapPoints, pointSize, 0.2f, 0.2f, 1);

		this.renderCameras(mapCamera, cameraShader, keyframes, mainPose, currentKeyframe, pointSize);
		this.renderCamera(mapCamera, cameraShader, (float) mainPose.getRotXDeg(), (float) mainPose.getRotYDeg(),
				(float) mainPose.getRotZDeg(), (float) mainPose.getCx(), (float) mainPose.getCy(),
				(float) mainPose.getCz(), pointSize, 1, 0.5f, 0);
	}

	public void renderMapView(Camera mapCamera, StaticShader cameraShader,
			HashMap<String, List<MapPoint>> mapPointsByCluster, List<MovingModel> movingObjects,
			List<MapPoint> trackedMapPoints, List<Keyframe> keyframes, Pose mainPose, Keyframe currentKeyframe,
			float pointSize, GUIComponents.COLOR_SCHEME colorScheme, ArUcoMap markerMap,
			List<Keyframe> closestKeyframes, List<Keyframe> loopDetectedKeyframes) {

		for (String key : mapPointsByCluster.keySet()) {

			if (partitionColors.get(key) == null) {
				Float[] color = new Float[3];
				color[0] = random.nextFloat() * 0.9f;
				color[1] = random.nextFloat() * 0.9f;
				color[2] = random.nextFloat() * 0.9f;
				partitionColors.put(key, color);
			}

			Float[] color;
			if (colorScheme == GUIComponents.COLOR_SCHEME.PARTITIONS) {
				color = partitionColors.get(key);
			} else {
				color = blue;
			}

			List<MapPoint> mapPoints = mapPointsByCluster.get(key);
			this.renderMapPoints(mapCamera, cameraShader, mapPoints, pointSize, color[0], color[1], color[2]);

		}

		// marker map points
		if (markerMap != null && markerMap.getMapPoints().size() > 0) {
			Transformation t = new Transformation(markerMap.getTransformation());
			Matrix tMatrix = t.getHomogeneousMatrix();
			float scale = (float) markerMap.getScale();
			List<Point3D> markerPoints = markerMap.getMapPoints().stream().map(mapPoint -> mapPoint.getPoint())
					.collect(Collectors.toList());

			List<Matrix> pointMatrices = new ArrayList<Matrix>();
			for (Point3D markerPoint : markerPoints) {
				Matrix m = new Matrix(4, 1, 1);
				m.set(0, 0, markerPoint.getX() * scale);
				m.set(1, 0, markerPoint.getY() * scale);
				m.set(2, 0, markerPoint.getZ() * scale);
				m = tMatrix.times(m);
				pointMatrices.add(m);
			}

			this.renderPointsMatrices(mapCamera, cameraShader, pointMatrices, pointSize, 0.6f, 0, 0.2f);

			List<Point3D> triangulatedMarkerPoints = markerMap.getMapPoints().stream()
					.map(mapPoint -> mapPoint.getTriangulatedPoint()).filter(point -> point != null)
					.collect(Collectors.toList());
//			this.renderPoints(mapCamera, cameraShader, t, scale, markerPoints, pointSize, 0.6f, 0, 0.2f);
			this.renderPoints(mapCamera, cameraShader, triangulatedMarkerPoints, pointSize, 1, 0.28f, 0);
		}

		this.renderMovingObjects(mapCamera, cameraShader, movingObjects, pointSize, 1, 0, 1);

		if (colorScheme == GUIComponents.COLOR_SCHEME.TRACKED) {
			Float[] color = red;
			this.renderMapPoints(mapCamera, cameraShader, trackedMapPoints, pointSize, color[0], color[1], color[2]);
		}

		this.renderCameras(mapCamera, cameraShader, keyframes, mainPose, currentKeyframe, pointSize);

		// loop detected keyframes (place recognition)
		this.renderCameras(mapCamera, cameraShader, loopDetectedKeyframes, mainPose, currentKeyframe, pointSize, 1, 0,
				1);

		// closest keyframes (place recognition)
		this.renderCameras(mapCamera, cameraShader, closestKeyframes, mainPose, currentKeyframe, pointSize, 0, 1, 0);

		this.renderCamera(mapCamera, cameraShader, (float) mainPose.getRotXDeg(), (float) mainPose.getRotYDeg(),
				(float) mainPose.getRotZDeg(), (float) mainPose.getCx(), (float) mainPose.getCy(),
				(float) mainPose.getCz(), pointSize, 1, 0.5f, 0);
	}

	public void renderMovingObjects(Camera mapCamera, StaticShader cameraShader, List<MovingModel> movingObjects,
			float pointSize, float r, float g, float b) {

		// organize points into matrices
		List<Matrix> points = new ArrayList<Matrix>();
		for (MovingModel mo : movingObjects) {
			Matrix p = new Matrix(4, mo.getMapPoints().size(), 1);
			for (int i = 0; i < mo.getMapPoints().size(); i++) {
				MapPoint mp = mo.getMapPoints().get(i);
				Point3D pt = mp.getPoint();
				if (pt == null) {
					Utils.pl("WARNING: A NULL POINT3D HAS BEEN DETECTED IN A MOVING OBJECT.");
					continue;
				}
				p.set(0, i, pt.getX());
				p.set(1, i, pt.getY());
				p.set(2, i, pt.getZ());
			}
			points.add(p);
		}

		// this may be worth refactoring (we can use the overloaded renderPoints(...)
		// method and pass the transformation/scale for OpenGL to handle)
		// transform points
		List<Matrix> transformedPoints = new ArrayList<Matrix>();
		for (int i = 0; i < points.size(); i++) {
			Matrix transformation = movingObjects.get(i).getTransformation().getHomogeneousMatrix();
			Matrix transformed = transformation.times(points.get(i));
			transformedPoints.add(transformed);
		}

		// render points
		this.renderPointsMatrices(mapCamera, cameraShader, transformedPoints, pointSize, r, g, b);
	}

	public void renderMapPoints(Camera mapCamera, StaticShader cameraShader, List<MapPoint> mapPoints, float pointSize,
			float r, float g, float b) {

		GL11.glPointSize(pointSize);

		GL11.glEnable(GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc(GL11.GL_LEQUAL);
		cameraShader.start();
		cameraShader.loadViewMatrix(mapCamera);
		cameraShader.loadCustomColor(new Vector3f(r, g, b));
		Matrix4f transformationMatrix = Maths.createTransformationMatrix(new Vector3f(0, 0, 0), 0, 0, 0, 1);
		cameraShader.loadTransformationMatrix(transformationMatrix);
		GL11.glBegin(GL11.GL_POINTS);
		for (int i = 0; i < mapPoints.size(); i++) {
			Point3D pt = mapPoints.get(i).getPoint();
			if (pt == null) {
				continue;
			}
			GL11.glVertex3d(pt.getX(), pt.getY(), pt.getZ());
		}
		GL11.glEnd();

		cameraShader.stop();
	}

	public void renderMapPoints(Camera mapCamera, StaticShader cameraShader, List<MapPoint> mapPoints,
			float pointSize) {

		GL11.glPointSize(pointSize);

		GL11.glEnable(GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc(GL11.GL_LEQUAL);
		cameraShader.start();
		cameraShader.loadViewMatrix(mapCamera);
		Matrix4f transformationMatrix = Maths.createTransformationMatrix(new Vector3f(0, 0, 0), 0, 0, 0, 1);
		cameraShader.loadTransformationMatrix(transformationMatrix);

		for (int i = 0; i < mapPoints.size(); i++) {
			Point3D pt = mapPoints.get(i).getPoint();
			if (pt == null) {
				continue;
			}
			float[] rgb = getHashedColor(pt);
			cameraShader.loadCustomColor(new Vector3f(rgb[0], rgb[1], rgb[2]));
			GL11.glBegin(GL11.GL_POINTS);
			GL11.glVertex3d(pt.getX(), pt.getY(), pt.getZ());
			GL11.glEnd();
		}

		cameraShader.stop();
	}

	public void renderPoints(Camera mapCamera, StaticShader cameraShader, List<Point3D> mapPoints, float pointSize,
			float r, float g, float b) {

		GL11.glPointSize(pointSize);

		GL11.glEnable(GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc(GL11.GL_LEQUAL);
		cameraShader.start();
		cameraShader.loadViewMatrix(mapCamera);
		cameraShader.loadCustomColor(new Vector3f(r, g, b));
		Matrix4f transformationMatrix = Maths.createTransformationMatrix(new Vector3f(0, 0, 0), 0, 0, 0, 1);
		cameraShader.loadTransformationMatrix(transformationMatrix);
		GL11.glBegin(GL11.GL_POINTS);
		for (int i = 0; i < mapPoints.size(); i++) {
			GL11.glVertex3d(mapPoints.get(i).getX(), mapPoints.get(i).getY(), mapPoints.get(i).getZ());
		}
		GL11.glEnd();

		cameraShader.stop();
	}

	public void renderPointsMatrices(Camera mapCamera, StaticShader cameraShader, List<Matrix> matrices,
			float pointSize, float r, float g, float b) {

		GL11.glPointSize(pointSize);

		GL11.glEnable(GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc(GL11.GL_LEQUAL);
		cameraShader.start();
		cameraShader.loadViewMatrix(mapCamera);
		cameraShader.loadCustomColor(new Vector3f(r, g, b));
		Matrix4f transformationMatrix = Maths.createTransformationMatrix(new Vector3f(0, 0, 0), 0, 0, 0, 1);
		cameraShader.loadTransformationMatrix(transformationMatrix);
		GL11.glBegin(GL11.GL_POINTS);
		for (int i = 0; i < matrices.size(); i++) {
			Matrix m = matrices.get(i);
			for (int j = 0; j < m.getColumnDimension(); j++) {
				GL11.glVertex3d(m.get(0, j), m.get(1, j), m.get(2, j));
			}

		}
		GL11.glEnd();

		cameraShader.stop();
	}

	// it appears there is some issue with the OpenGL transformation matrix stuff
	// (the end projection is slightly off, possibly caused by lossy conversion to
	// degrees or float), so don't use this method.
	public void renderPoints(Camera mapCamera, StaticShader cameraShader, Transformation transformation, float scale,
			List<Point3D> points, float pointSize, float r, float g, float b) {

		GL11.glPointSize(pointSize);

		GL11.glEnable(GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc(GL11.GL_LEQUAL);
		cameraShader.start();
		cameraShader.loadViewMatrix(mapCamera);
		cameraShader.loadCustomColor(new Vector3f(r, g, b));
		Matrix4f transformationMatrix = Maths.createTransformationMatrix(
				new Vector3f((float) transformation.getCx(), (float) transformation.getCy(),
						(float) transformation.getCz()),
				(float) transformation.getRotXDeg(), (float) transformation.getRotYDeg(),
				(float) transformation.getRotZDeg(), scale);
		cameraShader.loadTransformationMatrix(transformationMatrix);
		GL11.glBegin(GL11.GL_POINTS);
		for (int i = 0; i < points.size(); i++) {
			if (points.get(i) == null) {
				continue;
			}
			GL11.glVertex3d(points.get(i).getX(), points.get(i).getY(), points.get(i).getZ());
		}
		GL11.glEnd();

		cameraShader.stop();
	}

	public void renderCameras(Camera mapCamera, StaticShader cameraShader, List<Keyframe> keyframes, Pose currentPose,
			Keyframe currentKeyframe, float lineWidth) {
		renderCameras(mapCamera, cameraShader, keyframes, currentPose, currentKeyframe, lineWidth, 1, 0, 0);
	}

	public void renderCameras(Camera mapCamera, StaticShader cameraShader, List<Keyframe> keyframes, Pose currentPose,
			Keyframe currentKeyframe, float lineWidth, float r, float g, float b) {
		for (int i = 0; i < keyframes.size(); i++) {
			Keyframe kf = keyframes.get(i);
			Keyframe previousKeyframe = kf.getPreviousKeyframe();
			Pose pose = kf.getPose();
			this.renderCamera(mapCamera, cameraShader, (float) pose.getRotXDeg(), (float) pose.getRotYDeg(),
					(float) pose.getRotZDeg(), (float) pose.getCx(), (float) pose.getCy(), (float) pose.getCz(),
					lineWidth, r, g, b);

			// lines connecting keyframes
			if (previousKeyframe != null) {
				this.renderLine(mapCamera, cameraShader, (float) pose.getCx(), (float) pose.getCy(),
						(float) pose.getCz(), (float) previousKeyframe.getPose().getCx(),
						(float) previousKeyframe.getPose().getCy(), (float) previousKeyframe.getPose().getCz(),
						lineWidth, 0, 1, 0);
			}

		}
		// line connecting pose to current keyframe
		if (currentKeyframe != null) {
			this.renderLine(mapCamera, cameraShader, (float) currentPose.getCx(), (float) currentPose.getCy(),
					(float) currentPose.getCz(), (float) currentKeyframe.getPose().getCx(),
					(float) currentKeyframe.getPose().getCy(), (float) currentKeyframe.getPose().getCz(), lineWidth, 0,
					1, 1);
		}

	}

	public void renderLine(Camera mapCamera, StaticShader cameraShader, float cx0, float cy0, float cz0, float cx1,
			float cy1, float cz1, float lineWidth, float R, float G, float B) {

		GL11.glLineWidth(lineWidth);
		GL11.glEnable(GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc(GL11.GL_LEQUAL);
		cameraShader.start();
		cameraShader.loadViewMatrix(mapCamera);
		cameraShader.loadCustomColor(new Vector3f(R, G, B));
		Matrix4f transformationMatrix = Maths.createTransformationMatrix(new Vector3f(0, 0, 0), 0, 0, 0, 1);
		cameraShader.loadTransformationMatrix(transformationMatrix);

		GL11.glBegin(GL11.GL_LINES);
		GL11.glVertex3f(cx0, cy0, cz0);
		GL11.glVertex3f(cx1, cy1, cz1);
		GL11.glEnd();

		cameraShader.stop();

	}

	public void renderCamera(Camera mapCamera, StaticShader cameraShader, float rx, float ry, float rz, float Cx,
			float Cy, float Cz, float lineWidth, float R, float G, float B) {
		// (RGB values from 0 to 1)

		float wBy2 = Parameters.<Integer>get("width") / 2;
		float hBy2 = Parameters.<Integer>get("height") / 2;
		float f = Parameters.<Float>get("fx");

		GL11.glLineWidth(lineWidth);
		GL11.glEnable(GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc(GL11.GL_LEQUAL);
		cameraShader.start();
		cameraShader.loadViewMatrix(mapCamera);
		cameraShader.loadCustomColor(new Vector3f(R, G, B));
		Matrix4f transformationMatrix = Maths.createTransformationMatrix(new Vector3f(Cx, Cy, Cz), rx, ry, rz, 0.002f);
		cameraShader.loadTransformationMatrix(transformationMatrix);

		GL11.glBegin(GL11.GL_LINES);
		GL11.glVertex3f(0, 0, 0);
		GL11.glVertex3f(wBy2, hBy2, f);
		GL11.glVertex3f(0, 0, 0);
		GL11.glVertex3f(-wBy2, hBy2, f);
		GL11.glVertex3f(0, 0, 0);
		GL11.glVertex3f(-wBy2, -hBy2, f);
		GL11.glVertex3f(0, 0, 0);
		GL11.glVertex3f(wBy2, -hBy2, f);

		GL11.glVertex3f(wBy2, hBy2, f);
		GL11.glVertex3f(-wBy2, hBy2, f);
		GL11.glVertex3f(-wBy2, hBy2, f);
		GL11.glVertex3f(-wBy2, -hBy2, f);
		GL11.glVertex3f(-wBy2, -hBy2, f);
		GL11.glVertex3f(wBy2, -hBy2, f);
		GL11.glVertex3f(wBy2, -hBy2, f);
		GL11.glVertex3f(wBy2, hBy2, f);
		GL11.glEnd();

		cameraShader.stop();
	}

	private void createProjectionMatrix() {
		// float aspectRatio = (float) Display.getWidth() / (float) Display.getHeight();
		// float y_scale = (float) ((1f / Math.tan(Math.toRadians(FOV / 2f))) *
		// aspectRatio);
		// float y_scale = (float) ((1f / (Display.getWidth() / (2 *
		// CameraIntrinsics.fx))) * aspectRatio);
		// float x_scale = y_scale / aspectRatio;
		float frustum_length = FAR_PLANE - NEAR_PLANE;

		projectionMatrix = new Matrix4f();
		float width = Parameters.<Integer>get("width").floatValue();
		float height = Parameters.<Integer>get("height").floatValue();
		float x0 = 0; // 9
		float y0 = 0; // 15
//		projectionMatrix.m00(2 * CameraIntrinsics.fx / width);
//		projectionMatrix.m10(-2 * CameraIntrinsics.s / width);
//		projectionMatrix.m20((width - 2 * CameraIntrinsics.cx + 2 * x0) / width);
//		projectionMatrix.m11(2 * CameraIntrinsics.fy / height);
//		projectionMatrix.m21((-height + 2 * CameraIntrinsics.cy + 2 * y0) / height);
//		projectionMatrix.m22((-FAR_PLANE - NEAR_PLANE) / (FAR_PLANE - NEAR_PLANE));
//		projectionMatrix.m32(-2 * FAR_PLANE * NEAR_PLANE / frustum_length);
//		projectionMatrix.m23(-1);
//		projectionMatrix.m33(0);

//		// this didn't work
//		float cx = Parameters.width / 2.0f;
//		float cy = Parameters.height / 2.0f;

//		// y up and flipped z
//		projectionMatrix.m00(2 * (Float) Parameters.get("fx") / width);
//		projectionMatrix.m10(-2 * (Float) Parameters.get("s") / width);
//		projectionMatrix.m20((width - 2 * (Float) Parameters.get("cx") + 2 * x0) / width);
//		projectionMatrix.m11(-2 * (Float) Parameters.get("fy") / height);
//		projectionMatrix.m21((height - 2 * (Float) Parameters.get("cy") + 2 * y0) / height);
//		projectionMatrix.m22((FAR_PLANE + NEAR_PLANE) / frustum_length);
//		projectionMatrix.m32(-2 * FAR_PLANE * NEAR_PLANE / frustum_length);
//		projectionMatrix.m23(1);
//		projectionMatrix.m33(0);

		// http://kgeorge.github.io/2014/03/08/calculating-opengl-perspective-matrix-from-opencv-intrinsic-matrix
		projectionMatrix.m00(Parameters.<Float>get("fx") / Parameters.<Float>get("cx"));
		projectionMatrix.m10(0);
		projectionMatrix.m20(0);
		projectionMatrix.m30(0);
		projectionMatrix.m01(0);
		projectionMatrix.m11(-Parameters.<Float>get("fy") / Parameters.<Float>get("cy")); // added negative here
		projectionMatrix.m21(0);
		projectionMatrix.m31(0);
		projectionMatrix.m02(0);
		projectionMatrix.m12(0);
		projectionMatrix.m22((FAR_PLANE + NEAR_PLANE) / frustum_length); // removed negative here
		projectionMatrix.m32(-2 * FAR_PLANE * NEAR_PLANE / frustum_length);
		projectionMatrix.m03(0);
		projectionMatrix.m13(0);
		projectionMatrix.m23(1); // removed negative here
		projectionMatrix.m33(0);

	}

}
