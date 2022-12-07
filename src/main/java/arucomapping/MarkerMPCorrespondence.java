package arucomapping;

public class MarkerMPCorrespondence {

	protected ArUcoMapPoint mapPoint;
	protected ArUcoKeypoint keypoint;

	public MarkerMPCorrespondence() {

	}

	public ArUcoMapPoint getMapPoint() {
		return mapPoint;
	}

	public void setMapPoint(ArUcoMapPoint mapPoint) {
		this.mapPoint = mapPoint;
	}

	public ArUcoKeypoint getKeypoint() {
		return keypoint;
	}

	public void setKeypoint(ArUcoKeypoint keypoint) {
		this.keypoint = keypoint;
	}

}
