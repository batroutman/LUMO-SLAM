package gui;

import org.lwjgl.glfw.GLFW;

import types.Pose;

public class InputHandler {

	long lastMovement = 0;
	int movementFrametime = 10;
	long window = 0;

	double moveSpeed = 0.05;
	double rotateSpeed = 0.005;

	public InputHandler() {
	}

	public InputHandler(long window) {
		this.window = window;
	}

	// transform map camera to navigate map
	public void moveMapCamera(Pose mapTransformation) {

		long timestamp = System.currentTimeMillis();

		// if not enough time has passed for a sample, skip this frame
		if (timestamp - lastMovement < movementFrametime) {
			return;
		}

		if (GLFW.glfwGetKey(window, GLFW.GLFW_KEY_W) == 1) {
			// move forward
			mapTransformation.setT(mapTransformation.getTx(), mapTransformation.getTy(),
					mapTransformation.getTz() - moveSpeed);
		}
		if (GLFW.glfwGetKey(window, GLFW.GLFW_KEY_S) == 1) {
			// move backward
			mapTransformation.setT(mapTransformation.getTx(), mapTransformation.getTy(),
					mapTransformation.getTz() + moveSpeed);
		}
		if (GLFW.glfwGetKey(window, GLFW.GLFW_KEY_A) == 1) {
			// strafe left
			mapTransformation.setT(mapTransformation.getTx() + moveSpeed, mapTransformation.getTy(),
					mapTransformation.getTz());
		}
		if (GLFW.glfwGetKey(window, GLFW.GLFW_KEY_D) == 1) {
			// strafe right
			mapTransformation.setT(mapTransformation.getTx() - moveSpeed, mapTransformation.getTy(),
					mapTransformation.getTz());
		}
		if (GLFW.glfwGetKey(window, GLFW.GLFW_KEY_SPACE) == 1) {
			// move upward
			mapTransformation.setT(mapTransformation.getTx(), mapTransformation.getTy() + moveSpeed,
					mapTransformation.getTz());
		}
		if (GLFW.glfwGetKey(window, GLFW.GLFW_KEY_LEFT_SHIFT) == 1) {
			// move downward
			mapTransformation.setT(mapTransformation.getTx(), mapTransformation.getTy() - moveSpeed,
					mapTransformation.getTz());
		}

		if (GLFW.glfwGetKey(window, GLFW.GLFW_KEY_LEFT) == 1) {
			// rotate left
			mapTransformation.rotateEuler(0, rotateSpeed, 0);
		}

		if (GLFW.glfwGetKey(window, GLFW.GLFW_KEY_RIGHT) == 1) {
			// rotate right
			mapTransformation.rotateEuler(0, -rotateSpeed, 0);
		}

		if (GLFW.glfwGetKey(window, GLFW.GLFW_KEY_UP) == 1) {
			// rotate up
			mapTransformation.rotateEuler(-rotateSpeed, 0, 0);
		}

		if (GLFW.glfwGetKey(window, GLFW.GLFW_KEY_DOWN) == 1) {
			// rotate up
			mapTransformation.rotateEuler(rotateSpeed, 0, 0);
		}

		if (GLFW.glfwGetKey(window, GLFW.GLFW_KEY_COMMA) == 1) {
			// roll counter-clockwise
			mapTransformation.rotateEuler(0, 0, rotateSpeed);
		}

		if (GLFW.glfwGetKey(window, GLFW.GLFW_KEY_PERIOD) == 1) {
			// roll counter-clockwise
			mapTransformation.rotateEuler(0, 0, -rotateSpeed);
		}

		if (GLFW.glfwGetKey(window, GLFW.GLFW_KEY_K) == 1) {
			// increase move speed and rotate speed
			rotateSpeed *= 1.1;
			moveSpeed *= 1.1;
		}

		if (GLFW.glfwGetKey(window, GLFW.GLFW_KEY_J) == 1) {
			// decrease move speed and rotate speed
			rotateSpeed /= rotateSpeed < 0.00001 ? 1 : 1.1;
			moveSpeed /= moveSpeed < 0.0005 ? 1 : 1.1;
		}

		this.lastMovement = timestamp;

	}

	public long getLastMovement() {
		return lastMovement;
	}

	public void setLastMovement(long lastMovement) {
		this.lastMovement = lastMovement;
	}

	public int getMovementFrametime() {
		return movementFrametime;
	}

	public void setMovementFrametime(int movementFrametime) {
		this.movementFrametime = movementFrametime;
	}

	public long getWindow() {
		return window;
	}

	public void setWindow(long window) {
		this.window = window;
	}

}
