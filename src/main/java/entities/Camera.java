package entities;

import org.joml.Matrix4f;

import Jama.Matrix;
import toolbox.Utils;

public class Camera {

	// extrinsic format
	float r00 = 1;
	float r01 = 0;
	float r02 = 0;
	float tx = 0;
	float r10 = 0;
	float r11 = 1;
	float r12 = 0;
	float ty = 0;
	float r20 = 0;
	float r21 = 0;
	float r22 = 1;
	float tz = 0;

	public Camera() {
	}

	public void setMatrix(double r00, double r01, double r02, double r10, double r11, double r12, double r20,
			double r21, double r22, double tx, double ty, double tz) {
		this.r00 = (float) r00;
		this.r01 = (float) r01;
		this.r02 = (float) r02;
		this.r10 = (float) r10;
		this.r11 = (float) r11;
		this.r12 = (float) r12;
		this.r20 = (float) r20;
		this.r21 = (float) r21;
		this.r22 = (float) r22;
		this.tx = (float) tx;
		this.ty = (float) ty;
		this.tz = (float) tz;

	}

	public Matrix4f getViewMatrix() {

//		Matrix Rx = Matrix.identity(4, 4);
//
//		Rx.set(1, 1, Math.cos(Math.PI));
//		Rx.set(2, 2, Math.cos(Math.PI));
//		Rx.set(1, 2, -Math.sin(Math.PI));
//		Rx.set(2, 1, Math.sin(Math.PI));
//
//		Matrix Ry = Matrix.identity(4, 4);
//
//		Ry.set(0, 0, Math.cos(Math.PI));
//		Ry.set(2, 2, Math.cos(Math.PI));
//		Ry.set(2, 0, -Math.sin(Math.PI));
//		Ry.set(0, 2, Math.sin(Math.PI));

		Matrix mat = Matrix.identity(4, 4);
		mat.set(0, 0, r00);
		mat.set(0, 1, r01);
		mat.set(0, 2, r02);
		mat.set(1, 0, r10);
		mat.set(1, 1, r11);
		mat.set(1, 2, r12);
		mat.set(2, 0, r20);
		mat.set(2, 1, r21);
		mat.set(2, 2, r22);

		mat.set(0, 3, tx);
		mat.set(1, 3, ty);
		mat.set(2, 3, tz);

		Matrix4f viewMatrix = Utils.MatrixToMatrix4f(mat);
//		Matrix4f viewMatrix = Utils.MatrixToMatrix4f(Rx.times(mat));
//		Matrix4f viewMatrix = Utils.MatrixToMatrix4f(mat.times(Rx));
//		Utils.printMatrix(viewMatrix);
//		viewMatrix.transpose(viewMatrix);

		return viewMatrix;
	}

	public float getR00() {
		return r00;
	}

	public void setR00(float r00) {
		this.r00 = r00;
	}

	public float getR01() {
		return r01;
	}

	public void setR01(float r01) {
		this.r01 = r01;
	}

	public float getR02() {
		return r02;
	}

	public void setR02(float r02) {
		this.r02 = r02;
	}

	public float getTx() {
		return tx;
	}

	public void setTx(float tx) {
		this.tx = tx;
	}

	public float getR10() {
		return r10;
	}

	public void setR10(float r10) {
		this.r10 = r10;
	}

	public float getR11() {
		return r11;
	}

	public void setR11(float r11) {
		this.r11 = r11;
	}

	public float getR12() {
		return r12;
	}

	public void setR12(float r12) {
		this.r12 = r12;
	}

	public float getTy() {
		return ty;
	}

	public void setTy(float ty) {
		this.ty = ty;
	}

	public float getR20() {
		return r20;
	}

	public void setR20(float r20) {
		this.r20 = r20;
	}

	public float getR21() {
		return r21;
	}

	public void setR21(float r21) {
		this.r21 = r21;
	}

	public float getR22() {
		return r22;
	}

	public void setR22(float r22) {
		this.r22 = r22;
	}

	public float getTz() {
		return tz;
	}

	public void setTz(float tz) {
		this.tz = tz;
	}

}
