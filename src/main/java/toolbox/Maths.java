package toolbox;

import org.joml.Matrix4f;
import org.joml.Vector3f;

public class Maths {

	public static Matrix4f createTransformationMatrix(Vector3f translation, float rx, float ry, float rz, float scale) {
		Matrix4f matrix = new Matrix4f();
		matrix.identity();
		matrix.translate(translation, matrix);
		matrix.rotate((float) Math.toRadians(rx), new Vector3f(1, 0, 0), matrix);
		matrix.rotate((float) Math.toRadians(ry), new Vector3f(0, 1, 0), matrix);
		matrix.rotate((float) Math.toRadians(rz), new Vector3f(0, 0, 1), matrix);
		matrix.scale(new Vector3f(scale, scale, scale), matrix);
		return matrix;
	}

	public static Matrix4f createRotatedTransformationMatrix(Vector3f translation, float rx, float ry, float rz,
			float scale) {
		Matrix4f matrix = new Matrix4f();
		matrix.identity();
		matrix.translate(translation, matrix);
		matrix.rotate((float) Math.toRadians(rx), new Vector3f(1, 0, 0), matrix);
		matrix.rotate((float) Math.toRadians(ry), new Vector3f(0, 1, 0), matrix);
		matrix.rotate((float) Math.toRadians(rz), new Vector3f(0, 0, 1), matrix);
		matrix.rotate((float) Math.PI, new Vector3f(0, 0, 1), matrix);
		matrix.scale(new Vector3f(scale, scale, scale), matrix);
		return matrix;
	}

//	// unused
//	public static Matrix4f createViewMatrix(Camera camera) {
//		Matrix4f viewMatrix = new Matrix4f();
//		viewMatrix.identity();
//		viewMatrix.rotate((float) Math.toRadians(camera.getPitch()), new Vector3f(1, 0, 0), viewMatrix);
//		viewMatrix.rotate((float) Math.toRadians(camera.getYaw()), new Vector3f(0, 1, 0), viewMatrix);
//		Vector3f cameraPos = camera.getPosition();
//		Vector3f negativeCameraPos = new Vector3f(-cameraPos.x, -cameraPos.y, -cameraPos.z);
//		viewMatrix.translate(negativeCameraPos, viewMatrix);
//		return viewMatrix;
//	}

}
