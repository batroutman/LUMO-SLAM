package neuralnetwork;

import Jama.Matrix;

public class DenseLayer implements NetworkComponent {

	public int numInputs;
	public int numOutputs;

	public Matrix weights;
	public Matrix biases;
	public Matrix inputs;
	public Matrix outputs;
	public Matrix dWeights;
	public Matrix dBiases;
	public Matrix dInputs;

	public DenseLayer(int numInputs, int numOutputs) {
		this.weights = Matrix.random(numInputs, numOutputs);
		this.biases = new Matrix(1, numOutputs);
	}

	public void setWeights(Matrix weights) {
		this.weights = weights;
	}

	public void setBiases(Matrix biases) {
		this.biases = biases;
	}

	public Matrix getOutput() {
		return this.outputs;
	}

	// forward pass
	public void forward(Matrix inputs) {

		// remember inputs
		this.inputs = inputs;

		// calculate output values
		this.outputs = inputs.times(this.weights);

		// add biases
		for (int row = 0; row < this.outputs.getRowDimension(); row++) {
			this.outputs.setMatrix(row, row, 0, this.outputs.getColumnDimension() - 1,
					this.outputs.getMatrix(row, row, 0, this.outputs.getColumnDimension() - 1).plus(this.biases));
		}

	}

}
