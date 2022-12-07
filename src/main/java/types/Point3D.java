package types;

import Jama.Matrix;

public class Point3D {

	protected double x = 0;
	protected double y = 0;
	protected double z = 0;

	public Point3D() {

	}

	public Point3D(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public Point3D(Point3D p) {
		this.x = p.x;
		this.y = p.y;
		this.z = p.z;
	}

	public double getX() {
		return x;
	}

	public void setX(double x) {
		this.x = x;
	}

	public double getY() {
		return y;
	}

	public void setY(double y) {
		this.y = y;
	}

	public double getZ() {
		return z;
	}

	public void setZ(double z) {
		this.z = z;
	}

	public Matrix getHomogeneousMatrix() {
		Matrix X = new Matrix(4, 1);
		X.set(0, 0, this.x);
		X.set(1, 0, this.y);
		X.set(2, 0, this.z);
		X.set(3, 0, 1);
		return X;
	}

}
