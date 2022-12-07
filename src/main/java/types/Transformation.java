package types;

import Jama.Matrix;
import toolbox.Utils;

public class Transformation {

	double qw;
	double qx;
	double qy;
	double qz;

	double Cx;
	double Cy;
	double Cz;

	public Transformation() {

		qw = 1;
		qx = 0;
		qy = 0;
		qz = 0;

		Cx = 0;
		Cy = 0;
		Cz = 0;

	}

	public Transformation(Transformation transformation) {
		this.setTransformation(transformation);
	}

	public Transformation(Pose transformation) {
		this.setTransformation(transformation);
	}

	public void setTransformation(Transformation transformation) {
		qw = transformation.qw;
		qx = transformation.qx;
		qy = transformation.qy;
		qz = transformation.qz;

		Cx = transformation.Cx;
		Cy = transformation.Cy;
		Cz = transformation.Cz;
	}

	public void setTransformation(Pose transformation) {
		qw = transformation.qw;
		qx = transformation.qx;
		qy = transformation.qy;
		qz = transformation.qz;

		Cx = -transformation.Cx;
		Cy = -transformation.Cy;
		Cz = -transformation.Cz;
	}

	public double getDistanceFrom(Transformation transformation) {
		return Math.sqrt(Math.pow(this.Cx - transformation.getCx(), 2) + Math.pow(this.Cy - transformation.getCy(), 2)
				+ Math.pow(this.Cz - transformation.getCz(), 2));
	}

	public double getDistanceFrom(Pose transformation) {
		return Math.sqrt(Math.pow(this.Cx - transformation.getCx(), 2) + Math.pow(this.Cy - transformation.getCy(), 2)
				+ Math.pow(this.Cz - transformation.getCz(), 2));
	}

	public Matrix getHomogeneousMatrix() {
		Matrix R = this.getRotationMatrix();
		Matrix IC = Matrix.identity(4, 4);
		IC.set(0, 3, Cx);
		IC.set(1, 3, Cy);
		IC.set(2, 3, Cz);

		return R.times(IC);
	}

	public Transformation getReverseTransformation() {
		Transformation reverse = new Transformation();
		reverse.setQw(this.qw);
		reverse.setQx(-this.qx);
		reverse.setQy(-this.qy);
		reverse.setQz(-this.qz);
		reverse.setT(-this.Cx, -this.Cy, -this.Cz);
		return reverse;
	}

	public Matrix getRotationMatrix() {
		Matrix R = Matrix.identity(4, 4);
		R.set(0, 0, getR00());
		R.set(0, 1, getR01());
		R.set(0, 2, getR02());
		R.set(1, 0, getR10());
		R.set(1, 1, getR11());
		R.set(1, 2, getR12());
		R.set(2, 0, getR20());
		R.set(2, 1, getR21());
		R.set(2, 2, getR22());
		return R;
	}

	public void transform(Transformation T) {
		this.rotateQuat(T.qw, T.qx, T.qy, T.qz);
		this.translate(T.Cx, T.Cy, T.Cz);
	}

	public Matrix getQuaternion() {
		Matrix q = new Matrix(4, 1);
		q.set(0, 0, qw);
		q.set(1, 0, qx);
		q.set(2, 0, qy);
		q.set(3, 0, qz);
		return q;
	}

	public double getR00() {
		return 1 - 2 * qz * qz - 2 * qy * qy;
	}

	public double getR01() {
		return -2 * qz * qw + 2 * qy * qx;
	}

	public double getR02() {
		return 2 * qy * qw + 2 * qz * qx;
	}

	public double getTx() {
		return Cx * getR00() + Cy * getR01() + Cz * getR02();
	}

	public double getR10() {
		return 2 * qx * qy + 2 * qw * qz;
	}

	public double getR11() {
		return 1 - 2 * qz * qz - 2 * qx * qx;
	}

	public double getR12() {
		return 2 * qz * qy - 2 * qx * qw;
	}

	public double getTy() {
		return Cx * getR10() + Cy * getR11() + Cz * getR12();
	}

	public double getR20() {
		return 2 * qx * qz - 2 * qw * qy;
	}

	public double getR21() {
		return 2 * qy * qz + 2 * qw * qx;
	}

	public double getR22() {
		return 1 - 2 * qy * qy - 2 * qx * qx;
	}

	public double getTz() {
		return Cx * getR20() + Cy * getR21() + Cz * getR22();
	}

	public void normalize() {
		double mag = Math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
		qw /= mag;
		qx /= mag;
		qy /= mag;
		qz /= mag;
	}

	public double getQw() {
		return qw;
	}

	public void setQw(double qw) {
		this.qw = qw;
	}

	public double getQx() {
		return qx;
	}

	public void setQx(double qx) {
		this.qx = qx;
	}

	public double getQy() {
		return qy;
	}

	public void setQy(double qy) {
		this.qy = qy;
	}

	public double getQz() {
		return qz;
	}

	public void setQz(double qz) {
		this.qz = qz;
	}

	public double getCx() {
		return Cx;
	}

	public void setCx(double cx) {
		Cx = cx;
	}

	public double getCy() {
		return Cy;
	}

	public void setCy(double cy) {
		Cy = cy;
	}

	public double getCz() {
		return Cz;
	}

	public void setCz(double cz) {
		Cz = cz;
	}

	public double getRotX() {
		return Math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
	}

	public double getRotY() {
		return Math.asin(2 * (qw * qy - qz * qx));
	}

	public double getRotZ() {
		return Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
	}

	public double getRotXDeg() {
		return this.getRotX() * 180 / Math.PI;
	}

	public double getRotYDeg() {
		return this.getRotY() * 180 / Math.PI;
	}

	public double getRotZDeg() {
		return this.getRotZ() * 180 / Math.PI;
	}

	public void setT(double tx, double ty, double tz) {
		Matrix RInv = this.getRotationMatrix().getMatrix(0, 2, 0, 2).inverse();
		Matrix T = new Matrix(3, 1);
		T.set(0, 0, tx);
		T.set(1, 0, ty);
		T.set(2, 0, tz);
		Matrix C = RInv.times(T);
		this.Cx = C.get(0, 0);
		this.Cy = C.get(1, 0);
		this.Cz = C.get(2, 0);
	}

	public void rotateEuler(double x, double y, double z) {
		double yaw = z;
		double pitch = y;
		double roll = x;

		double cy = Math.cos(yaw * 0.5);
		double sy = Math.sin(yaw * 0.5);
		double cp = Math.cos(pitch * 0.5);
		double sp = Math.sin(pitch * 0.5);
		double cr = Math.cos(roll * 0.5);
		double sr = Math.sin(roll * 0.5);

		double qw = cr * cp * cy + sr * sp * sy;
		double qx = sr * cp * cy - cr * sp * sy;
		double qy = cr * sp * cy + sr * cp * sy;
		double qz = cr * cp * sy - sr * sp * cy;

		Matrix q2 = new Matrix(4, 1);
		q2.set(0, 0, qw);
		q2.set(1, 0, qx);
		q2.set(2, 0, qy);
		q2.set(3, 0, qz);

		Matrix newQ = Utils.quatMult(q2, this.getQuaternion());

		this.qw = newQ.get(0, 0);
		this.qx = newQ.get(1, 0);
		this.qy = newQ.get(2, 0);
		this.qz = newQ.get(3, 0);

		this.normalize();

	}

	// quaternion multiplication, assuming column vector of format [qw, qx, qy,
	// qz].transpose() (q1*q2)
	public void rotateQuat(double q1w, double q1x, double q1y, double q1z) {

		double q2w = this.qw;
		double q2x = this.qx;
		double q2y = this.qy;
		double q2z = this.qz;

		Matrix Q = new Matrix(4, 1);
		Q.set(1, 0, q1x * q2w + q1y * q2z - q1z * q2y + q1w * q2x);
		Q.set(2, 0, -q1x * q2z + q1y * q2w + q1z * q2x + q1w * q2y);
		Q.set(3, 0, q1x * q2y - q1y * q2x + q1z * q2w + q1w * q2z);
		Q.set(0, 0, -q1x * q2x - q1y * q2y - q1z * q2z + q1w * q2w);

		Q = Q.times(1 / Q.normF());

		this.qw = Q.get(0, 0);
		this.qx = Q.get(1, 0);
		this.qy = Q.get(2, 0);
		this.qz = Q.get(3, 0);
	}

	public void translate(double Cx, double Cy, double Cz) {
		this.Cx += Cx;
		this.Cy += Cy;
		this.Cz += Cz;
	}

	public String toString() {
		String output = "";
		output += "qw: " + this.qw + "\n";
		output += "qx: " + this.qx + "\n";
		output += "qy: " + this.qy + "\n";
		output += "qz: " + this.qz + "\n";
		output += "Cx: " + this.Cx + "\n";
		output += "Cy: " + this.Cy + "\n";
		output += "Cz: " + this.Cz + "\n";
		output += "tx: " + this.getTx() + "\n";
		output += "ty: " + this.getTy() + "\n";
		output += "tz: " + this.getTz() + "\n";
		return output;
	}

}
