package types;

public class Dbl {

	private double value;

	private Dbl() {

	}

	public Dbl(double value) {
		this.value = value;
	}

	public double getValue() {
		return value;
	}

	public void setValue(double value) {
		this.value = value;
	}

	public String toString() {
		return Double.toString(this.value);
	}

}
