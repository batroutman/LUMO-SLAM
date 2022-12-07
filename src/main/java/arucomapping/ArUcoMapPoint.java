package arucomapping;

import java.util.HashMap;

import lumoslam.Keyframe;
import types.Point3D;

public class ArUcoMapPoint {

	protected String id = "";
	protected Point3D point = null;
	protected Point3D triangulatedPoint = null;
	HashMap<Keyframe, ArUcoKeypoint> observations = new HashMap<Keyframe, ArUcoKeypoint>();

	public ArUcoMapPoint() {

	}

	public String getId() {
		return id;
	}

	public void setId(String id) {
		this.id = id;
	}

	public Point3D getPoint() {
		return point;
	}

	public void setPoint(Point3D point) {
		this.point = point;
	}

	public Point3D getTriangulatedPoint() {
		return triangulatedPoint;
	}

	public void setTriangulatedPoint(Point3D triangulatedPoint) {
		this.triangulatedPoint = triangulatedPoint;
	}

	public HashMap<Keyframe, ArUcoKeypoint> getObservations() {
		return observations;
	}

	public void setObservations(HashMap<Keyframe, ArUcoKeypoint> observations) {
		this.observations = observations;
	}

}
