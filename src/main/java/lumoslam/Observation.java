package lumoslam;

import org.opencv.core.Point;

import Jama.Matrix;
import types.Point3D;

public class Observation {

	protected Keyframe keyframe = null;
	protected Point point = null;
	protected Matrix normal = null;

	public Observation() {

	}

	public Observation(Keyframe keyframe, Point point, MapPoint mp) {
		this.keyframe = keyframe;
		this.point = point;
		this.calculateNormal(mp);
	}

	public void calculateNormal(MapPoint mp) {
		Point3D p = mp.getPoint();
		if (p == null || this.keyframe == null) {
			return;
		}
		Matrix normal = new Matrix(3, 1);
		normal.set(0, 0, p.getX() - this.keyframe.getPose().getCx());
		normal.set(1, 0, p.getY() - this.keyframe.getPose().getCy());
		normal.set(2, 0, p.getZ() - this.keyframe.getPose().getCz());
		normal = normal.times(1 / normal.normF());
		this.normal = normal;
	}

	public Keyframe getKeyframe() {
		return keyframe;
	}

	public void setKeyframe(Keyframe keyframe) {
		this.keyframe = keyframe;
	}

	public Point getPoint() {
		return point;
	}

	public void setPoint(Point point) {
		this.point = point;
	}

	public Matrix getNormal() {
		return normal;
	}

	public void setNormal(Matrix normal) {
		this.normal = normal;
	}

}
