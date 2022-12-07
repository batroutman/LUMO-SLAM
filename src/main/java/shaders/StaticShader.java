package shaders;

import org.joml.Matrix4f;
import org.joml.Vector3f;

import entities.Camera;

public class StaticShader extends ShaderProgram {

	public static final String VERTEX_FILE = "src/main/java/shaders/vertexShader.txt";
	public static final String FRAGMENT_FILE = "src/main/java/shaders/fragmentShader.txt";
	public static final String BG_VERTEX_FILE = "src/main/java/shaders/bgVertexShader.txt";
	public static final String COLOR_FRAGMENT_FILE = "src/main/java/shaders/colorFragmentShader.txt";

	private int location_transformationMatrix;
	private int location_projectionMatrix;
	private int location_viewMatrix;
	private int location_customColor;

	public StaticShader(String vertexFile, String fragmentFile) {
		super();
		this.loadShaders(vertexFile, fragmentFile);
//		if (background) {
//			this.loadShaders(BG_VERTEX_FILE, FRAGMENT_FILE);
//		} else {
//			this.loadShaders(VERTEX_FILE, FRAGMENT_FILE);
//		}
		this.setShaders();
	}

	@Override
	protected void bindAttributes() {
		super.bindAttribute(0, "position");
		super.bindAttribute(1, "textureCoordinates");
	}

	@Override
	protected void getAllUniformLocations() {
		location_transformationMatrix = super.getUniformLocation("transformationMatrix");
		location_projectionMatrix = super.getUniformLocation("projectionMatrix");
		location_viewMatrix = super.getUniformLocation("viewMatrix");
		location_customColor = super.getUniformLocation("customColor");

	}

	public void loadTransformationMatrix(Matrix4f matrix) {
		super.loadMatrix(location_transformationMatrix, matrix);
	}

	public void loadViewMatrix(Camera camera) {
		// Matrix4f viewMatrix = Maths.createViewMatrix(camera);
		Matrix4f viewMatrix = camera.getViewMatrix();
		super.loadMatrix(location_viewMatrix, viewMatrix);
	}

	public void loadProjectionMatrix(Matrix4f projection) {
		super.loadMatrix(location_projectionMatrix, projection);
	}

	public void loadCustomColor(Vector3f color) {
		super.loadVector(location_customColor, color);
	}

}
