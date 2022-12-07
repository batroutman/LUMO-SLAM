package neuralnetwork;

import Jama.Matrix;

public class ActivationReLU implements NetworkComponent {

	public Matrix inputs;
	public Matrix outputs;
	public Matrix dInputs;

	public ActivationReLU() {

	}

	public Matrix getOutput() {
		return this.outputs;
	}

	// forward pass
	public void forward(Matrix inputs) {

		// remember inputs
		this.inputs = inputs;

		// calculate output values
		this.outputs = clipNegatives(inputs);

	}

	// for matrix A, set all negative values in matrix to 0
	public static Matrix clipNegatives(Matrix A) {
		Matrix B = A.copy();
		for (int i = 0; i < B.getRowDimension(); i++) {
			for (int j = 0; j < B.getColumnDimension(); j++) {
				double original = B.get(i, j);
				B.set(i, j, original < 0 ? 0 : original);
			}
		}
		return B;
	}

}
