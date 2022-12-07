package placerecognition;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import toolbox.Timer;
import toolbox.Utils;

public class Vocabulary {

	public static Vocabulary global = new Vocabulary();

	public static class Node {
		public int id;
		public int parentId;
		public List<Integer> childrenIds;
		public List<Byte> descriptor;
		public double weight;
		public boolean isLeaf;
	}

	public List<Node> nodes = new ArrayList<Node>();
	public Node root = null;

	public void loadBoW(String filename) {

		Utils.pl("Loading descriptor vocabulary...");

		Timer t = new Timer();

		try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {

			this.nodes.clear();
			String line = null;
			boolean firstLine = true;

			// process each line one at a time
			while ((line = reader.readLine()) != null) {

				// skip first line
				if (firstLine) {
					firstLine = false;
					continue;
				}

				// build node
				String[] splits = line.split(" ");
				Node node = new Node();
				node.parentId = Integer.parseInt(splits[0]);
				node.isLeaf = Integer.parseInt(splits[1]) == 1;
				node.childrenIds = new ArrayList<Integer>();
				node.descriptor = new ArrayList<Byte>();
				for (int i = 2; i < 34; i++) {
					byte b = (byte) Integer.parseInt(splits[i]);
					node.descriptor.add(b);
				}
				node.weight = Double.parseDouble(splits[splits.length - 1]);

				this.nodes.add(node);

			}
		} catch (IOException x) {
			System.err.format("IOException: %s%n", x);
		}

		// set children and ids for each node
		for (int i = 0; i < this.nodes.size(); i++) {

			this.nodes.get(i).id = i;
			int parent = this.nodes.get(i).parentId;
			this.nodes.get(this.nodes.get(i).parentId).childrenIds.add(i);

		}

		// set root
		this.root = this.nodes.get(0);

		long millis = t.stop();

		Utils.pl("Vocabulary loaded (" + nodes.size() + " descriptors).");
		Utils.pl("(" + millis + " ms)");
	}

}
