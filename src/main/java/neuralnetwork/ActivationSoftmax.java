package neuralnetwork;

import Jama.Matrix;

public class ActivationSoftmax implements NetworkComponent {

	public Matrix inputs;
	public Matrix outputs;

	public ActivationSoftmax() {

	}

	public Matrix getOutput() {
		return this.outputs;
	}

	// forward pass
	public void forward(Matrix inputs) {

		// remember input values
		this.inputs = inputs;

		// get unnormalized probabilities
		Matrix expValues = expMatrix(this.inputs.plus(negativeRowMaxes(this.inputs)));

		this.outputs = getProbabilities(expValues);

	}

	// for n x m matrix A, return a matrix B of size n x m of the max values of each
	// row of A, multiplied by -1
	public static Matrix negativeRowMaxes(Matrix A) {

		Matrix B = new Matrix(A.getRowDimension(), A.getColumnDimension());

		for (int i = 0; i < A.getRowDimension(); i++) {
			double max = A.get(i, 0);
			for (int j = 0; j < A.getColumnDimension(); j++) {
				max = A.get(i, j) > max ? A.get(i, j) : max;
			}
			for (int j = 0; j < A.getColumnDimension(); j++) {
				B.set(i, j, -max);
			}
		}

		return B;

	}

	// for n x m matrix A, return n x m matrix B such that each element B(i,j) =
	// exp(A(i,j))
	public static Matrix expMatrix(Matrix A) {
		Matrix B = A.copy();

		for (int i = 0; i < B.getRowDimension(); i++) {
			for (int j = 0; j < B.getColumnDimension(); j++) {
				B.set(i, j, Math.exp(B.get(i, j)));
			}
		}

		return B;
	}

	// given an n x m matrix A, divide each element by the sum of its corresponding
	// row
	public static Matrix getProbabilities(Matrix A) {

		Matrix B = new Matrix(A.getRowDimension(), A.getColumnDimension());

		Matrix rowSums = new Matrix(A.getRowDimension(), 1);
		for (int j = 0; j < A.getColumnDimension(); j++) {
			rowSums = rowSums.plus(A.getMatrix(0, A.getRowDimension() - 1, j, j));
		}

		for (int i = 0; i < A.getRowDimension(); i++) {
			for (int j = 0; j < A.getColumnDimension(); j++) {
				B.set(i, j, A.get(i, j) / rowSums.get(i, 0));
			}
		}

		return B;

	}

}
