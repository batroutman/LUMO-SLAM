package gui;

import static org.lwjgl.glfw.GLFW.GLFW_KEY_ESCAPE;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_H;
import static org.lwjgl.glfw.GLFW.GLFW_RELEASE;
import static org.lwjgl.glfw.GLFW.glfwCreateWindow;
import static org.lwjgl.glfw.GLFW.glfwDestroyWindow;
import static org.lwjgl.glfw.GLFW.glfwMakeContextCurrent;
import static org.lwjgl.glfw.GLFW.glfwPollEvents;
import static org.lwjgl.glfw.GLFW.glfwSetWindowPos;
import static org.lwjgl.glfw.GLFW.glfwShowWindow;
import static org.lwjgl.glfw.GLFW.glfwSwapBuffers;
import static org.lwjgl.glfw.GLFW.glfwSwapInterval;
import static org.lwjgl.system.MemoryUtil.NULL;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.liquidengine.legui.DefaultInitializer;
import org.liquidengine.legui.animation.AnimatorProvider;
import org.liquidengine.legui.component.CheckBox;
import org.liquidengine.legui.component.Component;
import org.liquidengine.legui.component.Frame;
import org.liquidengine.legui.component.Label;
import org.liquidengine.legui.component.RadioButton;
import org.liquidengine.legui.component.RadioButtonGroup;
import org.liquidengine.legui.component.TextComponent;
import org.liquidengine.legui.component.Widget;
import org.liquidengine.legui.event.MouseClickEvent;
import org.liquidengine.legui.listener.MouseClickEventListener;
import org.liquidengine.legui.listener.processor.EventProcessorProvider;
import org.liquidengine.legui.style.color.ColorConstants;
import org.liquidengine.legui.system.context.CallbackKeeper;
import org.liquidengine.legui.system.context.Context;
import org.liquidengine.legui.system.layout.LayoutManager;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWKeyCallbackI;
import org.lwjgl.glfw.GLFWWindowCloseCallbackI;
import org.lwjgl.opengl.GL;
import org.lwjgl.opengl.GL11;

import runtimevars.Parameters;
import types.Callback;

public class GUIComponents {

	public long window;
	private Frame leguiframe;
	private DefaultInitializer initializer;
	private volatile boolean running = false;
	private volatile boolean hiding = false;
	private long lastUpdate = 0;
	private int updateInterval = 250;

//	public static enum COLOR_SCHEME {
//		DEFAULT, PARTITIONS, TRACKED
//	};

//	COLOR_SCHEME mapColor = COLOR_SCHEME.DEFAULT;

	private Widget mainWidget = new Widget(20, 20, 450, 400);

	private HashMap<String, Component> components = new HashMap<String, Component>();
	private HashMap<String, String> prefixes = new HashMap<String, String>();
	private HashMap<String, RadioButtonGroup> radioGroups = new HashMap<String, RadioButtonGroup>();

	private List<List<String>> componentOrder = new ArrayList<List<String>>();

	public GUIComponents() {

	}

	public void addLabel(String labelName, String prefix) {
		this.addLabel(labelName, prefix, 10, 10, 200, 10);
	}

	public void addLabel(String labelName, String prefix, float x, float y, float width, float height) {
		this.components.put(labelName, new Label(x, y, width, height));
		this.prefixes.put(labelName, prefix);
	}

	public void addRadioSet(String radioGroupName, Callback<MouseClickEvent> cb, String... buttonNames) {
		RadioButtonGroup group = new RadioButtonGroup();
		this.radioGroups.put(radioGroupName, group);
		for (String buttonName : buttonNames) {
			RadioButton button = new RadioButton(10, 10, 200, 10);
			button.getListenerMap().addListener(MouseClickEvent.class, (MouseClickEventListener) event -> {
				cb.callback(event);
			});
			button.setRadioButtonGroup(group);
			this.components.put(buttonName, button);
		}
	}

	public void addCheckbox(String checkboxName, String checkboxText, Boolean isChecked, Callback<MouseClickEvent> cb) {
		CheckBox checkbox = new CheckBox(10, 10, 200, 10);
		checkbox.getTextState().setText(checkboxText);
		checkbox.setChecked(isChecked);
		checkbox.getListenerMap().addListener(MouseClickEvent.class, (MouseClickEventListener) event -> {
			cb.callback(event);
		});
		this.components.put(checkboxName, checkbox);
	}

	public void initGUI() {
		if (!GLFW.glfwInit()) {
			throw new RuntimeException("Can't initialize GLFW");
		}
		window = createWindow();
		leguiframe = createFrameWithGUI();
		initializer = new DefaultInitializer(window, leguiframe);

		initializeGuiWithCallbacks();
		running = true;
	}

	private long createWindow() {
		long window = glfwCreateWindow(Parameters.<Integer>get("screenWidth"), Parameters.<Integer>get("screenHeight"),
				"LUMO-SLAM Demo Container", NULL, NULL);
		glfwShowWindow(window);
		glfwSetWindowPos(window, 0, 30);
		glfwMakeContextCurrent(window);
		GL.createCapabilities();
		glfwSwapInterval(0);
		return window;
	}

	// ADD GUI LABELS HERE
	public void initComponents() {

		this.addLabel("viewLabel", "View:");
		this.addRadioSet("viewButtonGroup", new Callback<MouseClickEvent>() {
			public void callback(MouseClickEvent e) {
				updateView();
			}
		}, "arViewButton", "processedViewButton", "mapViewButton", "allViewButton");
		this.prefixes.put("arViewButton", "AR View");
		this.prefixes.put("processedViewButton", "Processed View");
		this.prefixes.put("mapViewButton", "Map View");
		this.prefixes.put("allViewButton", "All Views");

		this.addLabel("featureDisplayLabel", "Feature Display Type:");
		this.addRadioSet("featureDisplayButtonGroup", new Callback<MouseClickEvent>() {
			public void callback(MouseClickEvent e) {
				updateFeatureDisplay();
			}
		}, "boxesButton", "pointsButton", "trackedKeypointsButton", "noneButton");
		this.prefixes.put("boxesButton", "Feature Boxes");
		this.prefixes.put("pointsButton", "Feature Points");
		this.prefixes.put("trackedKeypointsButton", "Tracked Points");
		this.prefixes.put("noneButton", "None");

		this.addLabel("mapColorLabel", "Map Color Scheme:");
		this.addRadioSet("mapColorButtonGroup", new Callback<MouseClickEvent>() {
			public void callback(MouseClickEvent e) {
				updateMapColor();
			}
		}, "mapColorDefaultButton", "mapColorTrackedButton", "mapColorPartitionsButton");
		this.prefixes.put("mapColorDefaultButton", "Default");
		this.prefixes.put("mapColorTrackedButton", "Tracked Points");
		this.prefixes.put("mapColorPartitionsButton", "Partitions");

		this.addLabel("trackingStatusLabel", "Tracking: ");
		this.addLabel("frameNumLabel", "Frame #: ");
		this.addLabel("fpsLabel", "Framerate: ");
		this.addLabel("numFeaturesLabel", "Number of Features: ");
		this.addLabel("numCorrespondencesLabel", "Number of Correspondences: ");
		this.addLabel("numPointsLabel", "Triangulated Points: ");
		this.addLabel("numKeyframesLabel", "Keyframes: ");
		this.addLabel("numClustersLabel", "Number of Clusters: ");
		this.addLabel("numMovingLabel", "Moving Objects: ");

		this.addLabel("arViewOptionsLabel", "AR View Options:");
		this.addCheckbox("showMapPointsBox", "Project Map Points", Parameters.<Boolean>get("ARView.showMapPoints"),
				new Callback<MouseClickEvent>() {
					public void callback(MouseClickEvent event) {
						Parameters.put("ARView.showMapPoints",
								((CheckBox) components.get("showMapPointsBox")).isChecked());
					}
				});
		this.addCheckbox("showBoundingBoxesBox", "Show Object Bounding Boxes",
				Parameters.<Boolean>get("ARView.showBoundingBoxes"), new Callback<MouseClickEvent>() {
					public void callback(MouseClickEvent event) {
						Parameters.put("ARView.showBoundingBoxes",
								((CheckBox) components.get("showBoundingBoxesBox")).isChecked());
					}
				});
		this.addCheckbox("showObjectLabelsBox", "Show Object Labels",
				Parameters.<Boolean>get("ARView.showObjectLabels"), new Callback<MouseClickEvent>() {
					public void callback(MouseClickEvent event) {
						Parameters.put("ARView.showObjectLabels",
								((CheckBox) components.get("showObjectLabelsBox")).isChecked());
					}
				});

		this.initLabelText();

		// set order
		List<String> col0 = new ArrayList<String>();
		col0.add("viewLabel");
		col0.add("arViewButton");
		col0.add("processedViewButton");
		col0.add("mapViewButton");
		col0.add("allViewButton");
		col0.add("featureDisplayLabel");
		col0.add("boxesButton");
		col0.add("pointsButton");
		col0.add("trackedKeypointsButton");
		col0.add("noneButton");
		col0.add("mapColorLabel");
		col0.add("mapColorDefaultButton");
		col0.add("mapColorTrackedButton");
		col0.add("mapColorPartitionsButton");

		List<String> col1 = new ArrayList<String>();
		col1.add("trackingStatusLabel");
		col1.add("frameNumLabel");
		col1.add("fpsLabel");
		col1.add("numFeaturesLabel");
		col1.add("numCorrespondencesLabel");
		col1.add("numPointsLabel");
		col1.add("numKeyframesLabel");
		col1.add("numClustersLabel");
		col1.add("numMovingLabel");
		col1.add("arViewOptionsLabel");
		col1.add("showMapPointsBox");
		col1.add("showBoundingBoxesBox");
		col1.add("showObjectLabelsBox");

		List<List<String>> order = new ArrayList<List<String>>();
		order.add(col0);
		order.add(col1);
		this.componentOrder = order;

	}

	public void initLabelText() {

		for (String labelName : this.prefixes.keySet()) {
			((TextComponent) this.components.get(labelName)).getTextState().setText(this.prefixes.get(labelName));
		}

	}

	// initialize the main widget that contains diagnostic data and options
	private Frame createFrameWithGUI() {
		Frame frame = new Frame(Parameters.<Integer>get("screenWidth").floatValue(),
				Parameters.<Integer>get("screenHeight").floatValue());

		this.initComponents();

		for (int i = 0; i < this.componentOrder.size(); i++) {
			for (int j = 0; j < this.componentOrder.get(i).size(); j++) {
				this.components.get(this.componentOrder.get(i).get(j)).setPosition(i * 200 + 10, 20 * j + 10);
			}
		}

		((RadioButton) this.components.get("arViewButton"))
				.setChecked(Parameters.<String>get("GUI.view").equalsIgnoreCase("AR"));
		((RadioButton) this.components.get("processedViewButton"))
				.setChecked(Parameters.<String>get("GUI.view").equalsIgnoreCase("PROCESSED"));
		((RadioButton) this.components.get("mapViewButton"))
				.setChecked(Parameters.<String>get("GUI.view").equalsIgnoreCase("MAP"));
		((RadioButton) this.components.get("allViewButton"))
				.setChecked(Parameters.<String>get("GUI.view").equalsIgnoreCase("ALL"));

		((RadioButton) this.components.get("boxesButton"))
				.setChecked(Parameters.<String>get("GUI.featureDisplayType").equalsIgnoreCase("BOXES"));
		((RadioButton) this.components.get("pointsButton"))
				.setChecked(Parameters.<String>get("GUI.featureDisplayType").equalsIgnoreCase("POINTS"));
		((RadioButton) this.components.get("trackedKeypointsButton"))
				.setChecked(Parameters.<String>get("GUI.featureDisplayType").equalsIgnoreCase("TRACKED_POINTS"));
		((RadioButton) this.components.get("noneButton"))
				.setChecked(Parameters.<String>get("GUI.featureDisplayType").equalsIgnoreCase("NONE"));

		((RadioButton) this.components.get("mapColorDefaultButton"))
				.setChecked(Parameters.<String>get("GUI.mapColorScheme").equalsIgnoreCase("DEFAULT"));
		((RadioButton) this.components.get("mapColorTrackedButton"))
				.setChecked(Parameters.<String>get("GUI.mapColorScheme").equalsIgnoreCase("TRACKED"));
		((RadioButton) this.components.get("mapColorPartitionsButton"))
				.setChecked(Parameters.<String>get("GUI.mapColorScheme").equalsIgnoreCase("PARTITIONS"));

		// Set background color for frame
		frame.getContainer().getStyle().getBackground().setColor(ColorConstants.transparent());
		frame.getContainer().setFocusable(false);

		this.mainWidget.getTitleTextState().setText("Data & Options");
		this.mainWidget.setCloseable(false);

		frame.getContainer().add(this.mainWidget);

		for (Component component : this.components.values()) {
			this.mainWidget.getContainer().add(component);
		}

		return frame;
	}

	private void initializeGuiWithCallbacks() {
		GLFWKeyCallbackI escapeCallback = (w1, key, code, action,
				mods) -> running = !(key == GLFW_KEY_ESCAPE && action != GLFW_RELEASE);

		// used to skip gui rendering
		GLFWKeyCallbackI hideCallback = (w1, key, code, action, mods) -> {
			if (key == GLFW_KEY_H && action == GLFW_RELEASE)
				hiding = !hiding;
		};
		GLFWWindowCloseCallbackI windowCloseCallback = w -> running = false;

		CallbackKeeper keeper = initializer.getCallbackKeeper();
		keeper.getChainKeyCallback().add(escapeCallback);
		keeper.getChainKeyCallback().add(hideCallback);
		keeper.getChainWindowCloseCallback().add(windowCloseCallback);

		org.liquidengine.legui.system.renderer.Renderer renderer = initializer.getRenderer();
		renderer.initialize();
	}

	public void renderGUI() {

		Context context = initializer.getContext();
		org.liquidengine.legui.system.renderer.Renderer ren = initializer.getRenderer();

		if (!hiding) {
			GL11.glViewport(0, 0, Parameters.<Integer>get("screenWidth"), Parameters.<Integer>get("screenHeight"));
			ren.render(leguiframe, context);
		}
		// poll events to callbacks
		glfwPollEvents();
		glfwSwapBuffers(window);
		// Now we need to process events. Firstly we need to process system
		// events.
		initializer.getSystemEventProcessor().processEvents(leguiframe, context);

		// When system events are translated to GUI events we need to
		// process them.
		// This event processor calls listeners added to ui components
		EventProcessorProvider.getInstance().processEvents();

		// When everything done we need to relayout components.
		LayoutManager.getInstance().layout(leguiframe);

		// Run animations. Should be also called cause some components use
		// animations for updating state.
		AnimatorProvider.getAnimator().runAnimations();
	}

	public void destroy() {
		initializer.getRenderer().destroy();
		glfwDestroyWindow(window);
	}

	public void updateView() {
		RadioButton selection = this.radioGroups.get("viewButtonGroup").getSelection();
		if (selection == this.components.get("arViewButton")) {
			Parameters.put("GUI.view", "AR");
		} else if (selection == this.components.get("processedViewButton")) {
			Parameters.put("GUI.view", "PROCESSED");
		} else if (selection == this.components.get("mapViewButton")) {
			Parameters.put("GUI.view", "MAP");
		} else if (selection == this.components.get("allViewButton")) {
			Parameters.put("GUI.view", "ALL");
		}
	}

	public void updateFeatureDisplay() {
		RadioButton selection = this.radioGroups.get("featureDisplayButtonGroup").getSelection();
		if (selection == this.components.get("boxesButton")) {
			Parameters.put("GUI.featureDisplayType", "BOXES");
		} else if (selection == this.components.get("pointsButton")) {
			Parameters.put("GUI.featureDisplayType", "POINTS");
		} else if (selection == this.components.get("trackedKeypointsButton")) {
			Parameters.put("GUI.featureDisplayType", "TRACKED_POINTS");
		} else if (selection == this.components.get("noneButton")) {
			Parameters.put("GUI.featureDisplayType", "NONE");
		}
	}

	public void updateMapColor() {
		RadioButton selection = this.radioGroups.get("mapColorButtonGroup").getSelection();
		if (selection == this.components.get("mapColorDefaultButton")) {
			Parameters.put("GUI.mapColorScheme", "DEAFULT");
		} else if (selection == this.components.get("mapColorTrackedButton")) {
			Parameters.put("GUI.mapColorScheme", "TRACKED");
		} else if (selection == this.components.get("mapColorPartitionsButton")) {
			Parameters.put("GUI.mapColorScheme", "PARTITIONS");
		}
	}

	// use this function for updating label text
	public void updateLabelText(String labelName, String text) {
		((TextComponent) this.components.get(labelName)).getTextState().setText(this.prefixes.get(labelName) + text);
	}

	public void updateFpsLabel(double fps) {
		long time = System.currentTimeMillis();
		if (time < this.lastUpdate + this.updateInterval) {
			return;
		}
		((TextComponent) this.components.get("fpsLabel")).getTextState()
				.setText(this.prefixes.get("fpsLabel") + String.format("%.2f", fps) + " fps");
		this.lastUpdate = time;
	}

	public long getWindow() {
		return window;
	}

	public void setWindow(long window) {
		this.window = window;
	}

	public Frame getLeguiframe() {
		return leguiframe;
	}

	public void setLeguiframe(Frame leguiframe) {
		this.leguiframe = leguiframe;
	}

	public DefaultInitializer getInitializer() {
		return initializer;
	}

	public void setInitializer(DefaultInitializer initializer) {
		this.initializer = initializer;
	}

	public boolean isRunning() {
		return running;
	}

	public void setRunning(boolean running) {
		this.running = running;
	}

	public boolean isHiding() {
		return hiding;
	}

	public void setHiding(boolean hiding) {
		this.hiding = hiding;
	}

	public Widget getMainWidget() {
		return mainWidget;
	}

	public void setMainWidget(Widget mainWidget) {
		this.mainWidget = mainWidget;
	}

}
