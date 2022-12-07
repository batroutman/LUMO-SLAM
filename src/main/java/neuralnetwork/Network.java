package neuralnetwork;

import java.util.ArrayList;
import java.util.List;

import org.json.JSONArray;

import Jama.Matrix;

public class Network {

	public List<NetworkComponent> components = new ArrayList<NetworkComponent>();

	public Network() {

	}

	public Network(String jsonArray) {
		this.initializeNetwork(jsonArray);
	}

	public Matrix feedForward(Matrix input) {
		Matrix lastResult = input;
		for (int i = 0; i < this.components.size(); i++) {
			this.components.get(i).forward(lastResult);
			lastResult = this.components.get(i).getOutput();
		}
		return lastResult;
	}

	// assuming the network will be a chain of ReLU activated dense layers ending in
	// a softmax output, build the ordered network components described by the
	// JSONArray, where each element of the json array if a JSONObject with
	// "weights" and "biases" arrays
	public void initializeNetwork(String jsonArray) {

		this.components.clear();

		JSONArray jsonComponents = new JSONArray(jsonArray);

		// build dense layers and ReLU activations
		for (int i = 0; i < jsonComponents.length(); i++) {
			JSONArray weights = jsonComponents.getJSONObject(i).getJSONArray("weights");
			JSONArray biases = jsonComponents.getJSONObject(i).getJSONArray("biases");
			int numInputs = weights.length();
			int numOutputs = weights.getJSONArray(0).length();
			DenseLayer dense = new DenseLayer(numInputs, numOutputs);
			dense.setWeights(JSONArrayToMatrix(weights));
			dense.setBiases(JSONArrayToMatrix(biases));

			this.components.add(dense);
			this.components.add(new ActivationReLU());
		}

		// remove the last ReLU activation and replace with softmax
		this.components.remove(this.components.size() - 1);
		this.components.add(new ActivationSoftmax());

	}

	public static Matrix JSONArrayToMatrix(JSONArray jsonArray) {

		int rows = jsonArray.length();
		int cols = jsonArray.getJSONArray(0).length();
		Matrix matrix = new Matrix(rows, cols);
		for (int row = 0; row < rows; row++) {
			JSONArray jsonRow = jsonArray.getJSONArray(row);
			for (int col = 0; col < cols; col++) {
				matrix.set(row, col, jsonRow.getNumber(col).doubleValue());
			}
		}

		return matrix;
	}

}
