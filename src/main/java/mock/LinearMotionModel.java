package mock;

import types.Transformation;

public class LinearMotionModel implements MotionModel {

	public static enum TranslationType {
		ABSOLUTE, RELATIVE
	};

	TranslationType translationType = TranslationType.ABSOLUTE;

	double rx = 0;
	double ry = 0;
	double rz = 0;

	double tx = 0;
	double ty = 0;
	double tz = 0;

	public LinearMotionModel() {

	}

	public LinearMotionModel(double rx, double ry, double rz, double tx, double ty, double tz,
			TranslationType translationType) {
		this.rx = rx;
		this.ry = ry;
		this.rz = rz;
		this.tx = tx;
		this.ty = ty;
		this.tz = tz;
		this.translationType = translationType;
	}

	public Transformation getTransformation(long numSteps) {
		Transformation transformation = new Transformation();
		transformation.rotateEuler(numSteps * this.rx, numSteps * this.ry, numSteps * this.rz);
		if (this.translationType == TranslationType.ABSOLUTE) {
			transformation.setCx(numSteps * tx);
			transformation.setCy(numSteps * ty);
			transformation.setCz(numSteps * tz);
		} else if (this.translationType == TranslationType.RELATIVE) {
			transformation.setT(numSteps * tx, numSteps * ty, numSteps * tz);
		}
		return transformation;
	}

	public TranslationType getTranslationType() {
		return translationType;
	}

	public void setTranslationType(TranslationType translationType) {
		this.translationType = translationType;
	}

	public double getRx() {
		return rx;
	}

	public void setRx(double rx) {
		this.rx = rx;
	}

	public double getRy() {
		return ry;
	}

	public void setRy(double ry) {
		this.ry = ry;
	}

	public double getRz() {
		return rz;
	}

	public void setRz(double rz) {
		this.rz = rz;
	}

	public double getTx() {
		return tx;
	}

	public void setTx(double tx) {
		this.tx = tx;
	}

	public double getTy() {
		return ty;
	}

	public void setTy(double ty) {
		this.ty = ty;
	}

	public double getTz() {
		return tz;
	}

	public void setTz(double tz) {
		this.tz = tz;
	}

}
