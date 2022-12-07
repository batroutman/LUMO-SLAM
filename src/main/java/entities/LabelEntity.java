package entities;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.joml.Vector3f;

import models.RawModel;
import models.TexturedModel;
import renderEngine.Glyph;
import renderEngine.Loader;
import textures.Font;
import textures.ModelTexture;
import toolbox.Utils;

public class LabelEntity extends Entity {

	protected Font fontAtlas;

	public LabelEntity(String label, Font fontAtlas, Vector3f position, float rotX, float rotY, float rotZ,
			float scale) {
		super();
		this.fontAtlas = fontAtlas;

		List<Float> verticesList = new ArrayList<Float>();
		List<Integer> indicesList = new ArrayList<Integer>();
		List<Float> textureCoordsList = new ArrayList<Float>();

		this.generateBuffers(label, verticesList, indicesList, textureCoordsList);

		float[] vertices = toFloatArray(verticesList);
		int[] indices = toIntArray(indicesList);
		float[] textureCoords = toFloatArray(textureCoordsList);

		RawModel rawModel = Loader.loadToVAO(vertices, textureCoords, indices);
		ModelTexture modelTexture = new ModelTexture(fontAtlas.getTexture().getId());
		TexturedModel texturedModel = new TexturedModel(rawModel, modelTexture);

		this.setModel(texturedModel);
		this.setPosition(position);
		this.setRotX(rotX);
		this.setRotY(rotY);
		this.setRotZ(rotZ);
		this.setScale(scale);

	}

	public void generateBuffers(String label, List<Float> vertices, List<Integer> indices, List<Float> textureCoords) {

		// get number of rows and columns (in terms of characters)
		int numCols = 0;
		int numRows = 1;
		for (int labelInd = 0, colInd = 1; labelInd < label.length(); labelInd++, colInd++) {
			char c = label.charAt(labelInd);
			if (c == '\n') {
				colInd = 0;
				numRows++;
			}
			if (colInd > numCols) {
				numCols = colInd;
			}
		}

		// generate vertices list
		vertices.clear();

		float colWidth = 0.5f;
		float rowHeight = 0.5f;

		float xOffset = colWidth * numCols / 2.0f;
		float yOffset = rowHeight * numRows / 2.0f;

		for (int row = 0; row < numRows; row++) {
			for (int col = 0; col < numCols; col++) {

				// x, y, z (top left)
				vertices.add(-col * colWidth + xOffset);
				vertices.add(-row * rowHeight + yOffset);
				vertices.add(0f);

				// x, y, z (top right)
				vertices.add(-(col + 1) * colWidth + xOffset);
				vertices.add(-row * rowHeight + yOffset);
				vertices.add(0f);

				// x, y, z (bottom left)
				vertices.add(-col * colWidth + xOffset);
				vertices.add(-(row + 1) * rowHeight + yOffset);
				vertices.add(0f);

				// x, y, z (bottom right)
				vertices.add(-(col + 1) * colWidth + xOffset);
				vertices.add(-(row + 1) * rowHeight + yOffset);
				vertices.add(0f);

			}
		}

		// generate indices list
		indices.clear();

		for (int row = 0; row < numRows; row++) {
			for (int col = 0; col < numCols; col++) {

				// bottom left
				indices.add(bottomLeftVertex(row, col, numCols));

				// bottom right
				indices.add(bottomRightVertex(row, col, numCols));

				// top right
				indices.add(topRightVertex(row, col, numCols));

				// top right
				indices.add(topRightVertex(row, col, numCols));

				// top left
				indices.add(topLeftVertex(row, col, numCols));

				// bottom left
				indices.add(bottomLeftVertex(row, col, numCols));

			}
		}

		// load string into table (string indices are "row,col")
		HashMap<String, Character> characterTable = new HashMap<String, Character>();
		for (int row = 0, col = 0, i = 0; i < label.length(); i++) {

			char c = label.charAt(i);

			if (c == '\n') {
				row++;
				col = 0;
				continue;
			}

			characterTable.put(row + "," + col, c);

			col++;
		}

		// generate texture coords list
		textureCoords.clear();

		int atlasWidth = this.fontAtlas.getAtlasWidth();
		int atlasHeight = this.fontAtlas.getAtlasHeight();
		Utils.pl("atlas height: " + atlasHeight);
		Utils.pl("atlas width: " + atlasWidth);

		for (int row = 0; row < numRows; row++) {
			for (int col = 0; col < numCols; col++) {

				Glyph glyph = this.fontAtlas.getGlyphs().get(characterTable.get(row + "," + col));

				Utils.pl(glyph);

				if (glyph == null) {
					textureCoords.add(0f);
					textureCoords.add(0f);
					textureCoords.add(0f);
					textureCoords.add(0f);
					textureCoords.add(0f);
					textureCoords.add(0f);
					textureCoords.add(0f);
					textureCoords.add(0f);
					continue;
				}

				// u, v (top left)
				textureCoords.add((float) glyph.x / atlasWidth);
				textureCoords.add((float) (glyph.height - glyph.y) / atlasHeight);

				// u, v (top right)
				textureCoords.add((float) (glyph.x + glyph.width) / atlasWidth);
				textureCoords.add((float) (glyph.height - glyph.y) / atlasHeight);

				// u, v (bottom left)
				textureCoords.add((float) glyph.x / atlasWidth);
				textureCoords.add((float) glyph.y / atlasHeight);

				// u, v (bottom right)
				textureCoords.add((float) (glyph.x + glyph.width) / atlasWidth);
				textureCoords.add((float) glyph.y / atlasHeight);

			}
		}

//		Utils.pl("\n\n\n\nLabelEntity stuff:\n\n");
//		Utils.pl("label: " + label);
//		Utils.pl("numRows, numCols: " + numRows + ", " + numCols);
//		Utils.pl("vertices: ");
//		Utils.pl(vertices);
//		Utils.pl("indices: ");
//		Utils.pl(indices);
//		Utils.pl("textureCoords: ");
//		Utils.pl(textureCoords);
//		Utils.pl("\n\n\n\n\n");

	}

	public static int topLeftVertex(int row, int col, int numCols) {

		return (row * numCols + col) * 4;

	}

	public static int topRightVertex(int row, int col, int numCols) {

		return topLeftVertex(row, col, numCols) + 1;

	}

	public static int bottomLeftVertex(int row, int col, int numCols) {

		return topLeftVertex(row, col, numCols) + 2;

	}

	public static int bottomRightVertex(int row, int col, int numCols) {

		return topLeftVertex(row, col, numCols) + 3;

	}

	public static float[] toFloatArray(List<Float> list) {
		float[] array = new float[list.size()];
		for (int i = 0; i < list.size(); i++) {
			array[i] = list.get(i);
		}
		return array;
	}

	public static int[] toIntArray(List<Integer> list) {
		int[] array = new int[list.size()];
		for (int i = 0; i < list.size(); i++) {
			array[i] = list.get(i);
		}
		return array;
	}

}
