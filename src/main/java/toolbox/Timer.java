package toolbox;

public class Timer {

	long begin = 0;
	long end = 0;

	public Timer() {
		this.start();
	}

	public void start() {
		this.begin = System.currentTimeMillis();
	}

	public long stop() {
		this.end = System.currentTimeMillis();
		return this.time();
	}

	// return time in milliseconds
	public long time() {
		return this.end - this.begin;
	}

	public double timeSec() {
		return this.time() / 1000.0;
	}

}
