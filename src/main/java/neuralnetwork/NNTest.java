package neuralnetwork;

import java.io.File;

import org.apache.commons.io.FileUtils;

public class NNTest {

	public static void test() {

		Network myNetwork = new Network(params8x8());
		System.out.println(myNetwork.components);

	}

	public static String params8x8() {
		String json = "";
		try {
			json = FileUtils.readFileToString(new File("data/json/8x8.json"), "utf-8");
		} catch (Exception e) {
			e.printStackTrace();
		}
		return json;
	}

	public static String params32x32() {
		String json = "";
		try {
			json = FileUtils.readFileToString(new File("data/json/32x32.json"), "utf-8");
		} catch (Exception e) {
			e.printStackTrace();
		}
		return json;
	}

}
