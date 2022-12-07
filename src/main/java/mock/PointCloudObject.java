package mock;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import Jama.Matrix;
import types.Transformation;

public class PointCloudObject extends MovingObject {

	// list of all 3D points (4x1 homogeneous vectors)
	List<Matrix> points = new ArrayList<Matrix>();

	public PointCloudObject() {

	}

	// given the frame number, return all 4x1 points transformed by the motion
	// models of the point cloud
	public List<Matrix> getTransformedPoints(long frameNum) {

		Transformation transformation = this.getTransformation(frameNum);
		Matrix tMatrix = transformation.getHomogeneousMatrix();

		List<Matrix> transformedPoints = new ArrayList<Matrix>();
		for (int i = 0; i < this.points.size(); i++) {
			Matrix tPoint = tMatrix.times(this.points.get(i));
			tPoint = tPoint.times(1 / tPoint.get(3, 0));
			transformedPoints.add(tPoint);
		}

		return transformedPoints;

	}

	public void addPlane(int seed, int numPoints, double x0, double y0, double z0, double normalX, double normalY,
			double normalZ, double xRange, double yRange, double zRange, double noiseRange) {

		List<Matrix> points = new ArrayList<Matrix>();

		// calculate minimum values for each dimension
		double xMin = x0 - xRange / 2;
		double yMin = y0 - yRange / 2;
		double zMin = z0 - zRange / 2;

		Random rand = new Random(seed);

		// generate points
		for (int i = 0; i < numPoints; i++) {

			double x = rand.nextDouble() * xRange + xMin;
			double y = rand.nextDouble() * yRange + yMin;
			double z = rand.nextDouble() * zRange + zMin;

			// correct one of the coordinates
			if (normalZ != 0) {
				z = (-normalX * (x - x0) - normalY * (y - y0)) / normalZ + z0;
			} else if (normalX != 0) {
				x = (-normalZ * (z - z0) - normalY * (y - y0)) / normalX + x0;
			} else if (normalY != 0) {
				y = (-normalZ * (z - z0) - normalX * (x - x0)) / normalY + y0;
			}

			double noiseX = rand.nextDouble() * noiseRange - noiseRange / 2;
			double noiseY = rand.nextDouble() * noiseRange - noiseRange / 2;
			double noiseZ = rand.nextDouble() * noiseRange - noiseRange / 2;

			x += noiseX;
			y += noiseY;
			z += noiseZ;

			Matrix p = new Matrix(4, 1);
			p.set(0, 0, x);
			p.set(1, 0, y);
			p.set(2, 0, z);
			p.set(3, 0, 1);
			points.add(p);

		}

		this.points.addAll(points);

	}

	public void addSphere(int seed, int numPoints, double x0, double y0, double z0, double radius, double noiseRange) {

		List<Matrix> points = new ArrayList<Matrix>();

		Random rand = new Random(seed);

		for (int i = 0; i < numPoints; i++) {
			double theta = rand.nextDouble() * 2 * Math.PI;
			double phi = rand.nextDouble() * Math.PI;

			double x = radius * Math.cos(theta) * Math.sin(phi) + x0;
			double y = radius * Math.sin(theta) * Math.sin(phi) + y0;
			double z = radius * Math.cos(phi) + z0;

			double noiseX = rand.nextDouble() * noiseRange - noiseRange / 2;
			double noiseY = rand.nextDouble() * noiseRange - noiseRange / 2;
			double noiseZ = rand.nextDouble() * noiseRange - noiseRange / 2;

			x += noiseX;
			y += noiseY;
			z += noiseZ;

			Matrix p = new Matrix(4, 1);
			p.set(0, 0, x);
			p.set(1, 0, y);
			p.set(2, 0, z);
			p.set(3, 0, 1);
			points.add(p);
		}

		this.points.addAll(points);

	}

}
