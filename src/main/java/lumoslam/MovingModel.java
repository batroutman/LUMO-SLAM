package lumoslam;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import Jama.Matrix;
import types.Point3D;
import types.Transformation;

public class MovingModel {

	public class BoundingBox {
		public double x0 = 0;
		public double x1 = 0;
		public double y0 = 0;
		public double y1 = 0;
		public double z0 = 0;
		public double z1 = 0;
	}

	String label = "";
	List<MapPoint> mapPoints = new ArrayList<MapPoint>();
	Transformation transformation = new Transformation();
	Point3D centroid = null;

	public MovingModel() {

	}

	public MovingModel(MovingModel mo) {
		this.label = mo.label;
		this.mapPoints = new ArrayList<MapPoint>(mo.mapPoints);
		this.transformation = new Transformation(mo.transformation);
		this.centroid = new Point3D(mo.centroid);
	}

	public BoundingBox getBoundingBox() {

		BoundingBox bb = new BoundingBox();
		if (this.mapPoints.size() <= 0) {
			return bb;
		}

		Matrix transformation = this.transformation.getHomogeneousMatrix();

		boolean initialized = false;
		for (int i = 0; i < this.mapPoints.size(); i++) {

			Point3D p = this.mapPoints.get(i).getPoint();
			if (p == null) {
				continue;
			}

			double x = p.getX();
			double y = p.getY();
			double z = p.getZ();

			Matrix pointMatrix = new Matrix(4, 1, 1);
			pointMatrix.set(0, 0, x);
			pointMatrix.set(1, 0, y);
			pointMatrix.set(2, 0, z);

			pointMatrix = transformation.times(pointMatrix);
			x = pointMatrix.get(0, 0);
			y = pointMatrix.get(1, 0);
			z = pointMatrix.get(2, 0);

			if (!initialized) {
				bb.x0 = x;
				bb.x1 = x;
				bb.y0 = y;
				bb.y1 = y;
				bb.z0 = z;
				bb.z1 = z;
				initialized = true;
			} else {
				bb.x0 = Math.min(x, bb.x0);
				bb.x1 = Math.max(x, bb.x1);
				bb.y0 = Math.min(y, bb.y0);
				bb.y1 = Math.max(y, bb.y1);
				bb.z0 = Math.min(z, bb.z0);
				bb.z1 = Math.max(z, bb.z1);
			}

		}

		return bb;

	}

	public Point3D calculateCentroid() {

		Point3D center = new Point3D(0, 0, 0);
		List<Point3D> points = this.mapPoints.stream().map(mapPoint -> mapPoint.getPoint())
				.filter(point -> point != null).collect(Collectors.toList());
		for (Point3D p : points) {
			center.setX(center.getX() + p.getX());
			center.setY(center.getY() + p.getY());
			center.setZ(center.getZ() + p.getZ());
		}
		center.setX(center.getX() / points.size());
		center.setY(center.getY() / points.size());
		center.setZ(center.getZ() / points.size());
		this.centroid = center;
		return this.centroid;

	}

	public Point3D getTransformedCentroid() {
		if (this.centroid == null) {
			return null;
		}
		Matrix p = new Matrix(4, 1, 1);
		p.set(0, 0, this.centroid.getX());
		p.set(1, 0, this.centroid.getY());
		p.set(2, 0, this.centroid.getZ());

		Matrix transform = this.transformation.getHomogeneousMatrix();
		Matrix tPoint = transform.times(p);
		Point3D transformedPoint = new Point3D(tPoint.get(0, 0), tPoint.get(1, 0), tPoint.get(2, 0));
		return transformedPoint;
	}

	public List<MapPoint> getMapPoints() {
		return mapPoints;
	}

	public void setMapPoints(List<MapPoint> mapPoints) {
		this.mapPoints = mapPoints;
	}

	public Transformation getTransformation() {
		return transformation;
	}

	public void setTransformation(Transformation transformation) {
		this.transformation = transformation;
	}

	public String getLabel() {
		return label;
	}

	public void setLabel(String label) {
		this.label = label;
	}

	public Point3D getCentroid() {
		return centroid;
	}

	public void setCentroid(Point3D centroid) {
		this.centroid = centroid;
	}

}
