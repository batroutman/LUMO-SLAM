package neuralnetwork;

import Jama.Matrix;

public interface NetworkComponent {

	public void forward(Matrix inputs);

	public Matrix getOutput();

}
