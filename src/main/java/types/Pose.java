package types;

import Jama.Matrix;
import toolbox.Utils;

// representation of a 3D geometric transformation, where the subject is first translated with (Cx, Cy, Cz),
// and then rotated with unit quaternion (qw, qx, qy, qz)
// the Pose class, however, represents this transformation for a camera. 
// Thus, the parameters are logically reversed, as the pinhole camera model's "camera" is a transformation 
// on all world points (if I could go back and remake this system, I would just use the Transformation class 
// and omit the Pose class altogether).
public class Pose {

	long timestamp;

	double qw;
	double qx;
	double qy;
	double qz;

	double Cx;
	double Cy;
	double Cz;

	boolean fixed = false;

	public Pose() {
		timestamp = System.nanoTime();

		qw = 1;
		qx = 0;
		qy = 0;
		qz = 0;

		Cx = 0;
		Cy = 0;
		Cz = 0;
	}

	public Pose(Pose pose) {
		this.setPose(pose);
	}

	public void setPose(Pose pose) {
		timestamp = pose.timestamp;

		qw = pose.qw;
		qx = pose.qx;
		qy = pose.qy;
		qz = pose.qz;

		Cx = pose.Cx;
		Cy = pose.Cy;
		Cz = pose.Cz;
	}

	public Point3D transformPoint(Point3D p) {
		Point3D transformedPoint = new Point3D();

		Matrix q = new Matrix(4, 1);
		q.set(0, 0, this.qw);
		q.set(1, 0, this.qx);
		q.set(2, 0, this.qy);
		q.set(3, 0, this.qz);

		Matrix qInv = new Matrix(4, 1);
		qInv.set(0, 0, this.qw);
		qInv.set(1, 0, -this.qx);
		qInv.set(2, 0, -this.qy);
		qInv.set(3, 0, -this.qz);

		Matrix pointMatrix = new Matrix(4, 1);
		pointMatrix.set(0, 0, 0);
		pointMatrix.set(1, 0, p.getX() - this.Cx);
		pointMatrix.set(2, 0, p.getY() - this.Cy);
		pointMatrix.set(3, 0, p.getZ() - this.Cz);

		Matrix result = Utils.quatMultPure(Utils.quatMultPure(q, pointMatrix), qInv);

		transformedPoint.setX(result.get(1, 0));
		transformedPoint.setY(result.get(2, 0));
		transformedPoint.setZ(result.get(3, 0));

		return transformedPoint;
	}

	/**
	 * Return a pose output such that output = p * this
	 * 
	 * @param p pose to transform this by
	 * @return result of p * this
	 */
	public Pose applyPose(Pose p) {
		Pose newPose = new Pose();

		Matrix q = new Matrix(4, 1);
		q.set(0, 0, p.qw);
		q.set(1, 0, p.qx);
		q.set(2, 0, p.qy);
		q.set(3, 0, p.qz);

		Matrix r = new Matrix(4, 1);
		r.set(0, 0, this.qw);
		r.set(1, 0, this.qx);
		r.set(2, 0, this.qy);
		r.set(3, 0, this.qz);

		Matrix resultQ = Utils.quatMult(q, r);

		newPose.setQw(resultQ.get(0, 0));
		newPose.setQx(resultQ.get(1, 0));
		newPose.setQy(resultQ.get(2, 0));
		newPose.setQz(resultQ.get(3, 0));
		newPose.setCx(this.Cx + p.getCx());
		newPose.setCy(this.Cy + p.getCy());
		newPose.setCz(this.Cz + p.getCz());

		return newPose;
	}

	/**
	 * Returns the similarity transform, ij, such that
	 * 
	 * this = ij * j
	 * 
	 * @param j Starting pose.
	 */
	public Pose similarityTransformFrom(Pose j) {
		Pose ij = new Pose();

		Matrix qi = new Matrix(4, 1);
		qi.set(0, 0, this.qw);
		qi.set(1, 0, this.qx);
		qi.set(2, 0, this.qy);
		qi.set(3, 0, this.qz);

		Matrix qjInv = new Matrix(4, 1);
		qjInv.set(0, 0, -j.qw);
		qjInv.set(1, 0, j.qx);
		qjInv.set(2, 0, j.qy);
		qjInv.set(3, 0, j.qz);

		Matrix q = Utils.quatMult(qi, qjInv);

		ij.qw = q.get(0, 0);
		ij.qx = q.get(1, 0);
		ij.qy = q.get(2, 0);
		ij.qz = q.get(3, 0);
		ij.Cx = this.Cx - j.Cx;
		ij.Cy = this.Cy - j.Cy;
		ij.Cz = this.Cz - j.Cz;

		return ij;
	}

	public double getDistanceFrom(Pose pose) {
		return Math.sqrt(Math.pow(this.Cx - pose.getCx(), 2) + Math.pow(this.Cy - pose.getCy(), 2)
				+ Math.pow(this.Cz - pose.getCz(), 2));
	}

	public Matrix getHomogeneousMatrix() {
		Matrix R = this.getRotationMatrix();
		Matrix IC = Matrix.identity(4, 4);
		IC.set(0, 3, -Cx);
		IC.set(1, 3, -Cy);
		IC.set(2, 3, -Cz);

		return R.times(IC);
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

	public Matrix getQuaternion() {
		Matrix q = new Matrix(4, 1);
		q.set(0, 0, qw);
		q.set(1, 0, qx);
		q.set(2, 0, qy);
		q.set(3, 0, qz);
		return q;
	}

	public long getTimestamp() {
		return timestamp;
	}

	public void setTimestamp(long timestamp) {
		this.timestamp = timestamp;
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
		return -(Cx * getR00() + Cy * getR01() + Cz * getR02());
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
		return -(Cx * getR10() + Cy * getR11() + Cz * getR12());
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
		return -(Cx * getR20() + Cy * getR21() + Cz * getR22());
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
		return -Math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
	}

	public double getRotY() {
		return -Math.asin(2 * (qw * qy - qz * qx));
	}

	public double getRotZ() {
		return -Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
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
		this.Cx = -C.get(0, 0);
		this.Cy = -C.get(1, 0);
		this.Cz = -C.get(2, 0);
	}

	public boolean isFixed() {
		return fixed;
	}

	public void setFixed(boolean fixed) {
		this.fixed = fixed;
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

	public String toString() {
		return String.format("Pose { qw: %f, qx: %f, qy: %f, qz: %f, Cx: %f, Cy: %f, Cz: %f }", this.qw, this.qx,
				this.qy, this.qz, this.Cx, this.Cy, this.Cz);
	}

}
