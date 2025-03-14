# LUMO-SLAM: Simultaneous Localization and Mapping with Localization of Unknown Moving Objects

LUMO-SLAM is a markerless, monocular SLAM system that can also detect and localize moving objects without knowledge of the objects' appearance, structure, or existence.

https://user-images.githubusercontent.com/17709606/207700356-9f148d86-9176-4f93-841a-bbf9ce1e53a9.mp4

For a condensed technical report covering the architecture and algorithms used in LUMO-SLAM, visit [https://web.ics.purdue.edu/~mtucerya/tech-reports/TR-CIS-0124-01.pdf](https://web.ics.purdue.edu/~mtucerya/tech-reports/TR-CIS-0124-01.pdf).

## Citing
If you would like to cite this work, you can reference the following technical report:
```
@techreport{Troutman2024LUMOSLAM,
    author      = {Troutman, Blake and Tuceryan, Mihran},
    title       = {{LUMO-SLAM}: A Solution for Monocular Dynamic {SLAM}},
    institution = {Purdue University},
    year        = {2024},
    type        = {Technical Report},
    number      = {TR-CIS-0124-01},
    address     = {Indianapolis, Indiana},
    month       = {January},
    url         = {https://web.ics.purdue.edu/~mtucerya/tech-reports/TR-CIS-0124-01.pdf}
}
```

Or, you can reference the original dissertation:
```
@phdthesis{TroutmanThesis2023,
    author = "Blake Austin Troutman",
    title = "{Registration and Localization of Unknown Moving Objects in Markerless Monocular SLAM}",
    school = "Purdue University, Indianapolis",
    year = "2023",
    month = "5",
    url = "https://hammer.purdue.edu/articles/thesis/Registration_and_Localization_of_Unknown_Moving_Objects_in_Markerless_Monocular_SLAM/22645324",
    doi = "10.25394/PGS.22645324.v1"
}
```

## Installation
This system is implemented in **Java 1.8**. Java is used instead of C++ in an effort to make it easier to port the system to android devices in the future (though there are currently no concrete plans to do so—feel free to adapt this system to android yourself if you need a mobile implementation of this system).

The system is structured as a Gradle-Eclipse project, with a few additional external libraries needed. So the easiest way to build the system will be to:
1. Download [Eclipse IDE](https://www.eclipse.org/downloads/)
2. Import the project with File > Import... > Gradle > Existing Gradle Project
3. Ensure the Gradle libraries are installed by right-clicking the project in the explorer menu (`LUMO-SLAM [LUMO-SLAM master]`) > Gradle > Refresh Gradle Project
4. Add the additional libraries and attach them to the project

The libraries required by the system are specified below.

## Gradle libraries
#### These libraries are included in the build.gradle file, so if you are installing with Eclipse/Gradle, you do not need to acquire these libraries manually and you can skip this part of the installation.

### lwjgl 3.2.3 - Lightweight Java Game Library
- https://www.lwjgl.org/
- lwjgl is a Java graphics library which, in this project, is used for its implementation of the OpenGL. 
- NOTE: ADD OTHER SYSTEM BINARIES TO GRADLE FILE FOR LWJGL

### Jama 1.0.2
- https://math.nist.gov/javanumerics/jama/
- Jama is a Java matrix library that provides useful implementations for matrix operations (arithmetic, SVD, etc.).

### legui 3.2.0
- https://github.com/SpinyOwl/legui
- legui is a basic GUI framework build on top of lwjgl 3. This is used for the options menu of the program.



## External libraries
#### These libraries are NOT included in the build.gradle file, and thus will have to be acquired and added to the project manually.

### OpenCV (with extra modules) 3.4.17
- https://github.com/opencv/opencv/tree/3.4
- https://github.com/opencv/opencv_contrib/tree/3.4 (extra modules, required)
- OpenCV is the library that handles almost all of the standard computer vision algorithms for this project.
- opencv_contrib is required for the ArUco marker tracking that is used when developing groundtruth data. Markers are otherwise not used in the typical operation of LUMO-SLAM system.

### BoofCV 0.36
- http://boofcv.org/index.php?title=Download
- BoofCV is an additional computer vision library that is used in this project for its bundle adjustment capabilities.

### JSON-java
- https://github.com/stleary/JSON-java
- The JSON-java library is used for parsing JSON data from configuration files.

## Additional requirements
### ORB BoW Vocabulary
- Since the addition of place recognition and loop closure, LUMO-SLAM uses the same bag-of-words ORB vocabulary that [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) uses, as LUMO-SLAM also uses ORB features.
- This vocabulary text file can be downloaded from this [mirror](https://files.blaketroutman.com/ORBvoc.txt), or can be downloaded as a result of installing [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2).
- After downloading this text file, it will need to be referenced in your config file (specified below).

## Running the System
### Configuration Files
The system currently parses 2 formats of video data: mp4 files and sequence data from the TUM RGB-D dataset (https://vision.in.tum.de/data/datasets/rgbd-dataset). Once you have a sequence downloaded, run the system with the `-config <config_file.json>` argument to specify input data. Config files take the following form:

```json
{
	"// INPUT DATA INFORMATION": "",
	"inputDataPath": "../datasets/rgbd_dataset_freiburg3_long_office_household/",
	"bufferType": "TUM",
	"bowVocabFile": "../BoW/ORBvoc.txt",
	
	"// TRAJECTORY SAVE INFORMATION": "(Output data info)",
	"trajectorySavePath": "outputs/trajectory.txt",
	"keyframeTrajectorySavePath": "outputs/kftrajectory.txt",
	
	"// CAMERA INTRINSIC PARAMETERS": "",
	"fx": 535.4,
	"fy": 539.2,
	"cx": 320.1,
	"cy": 247.6,
	"s": 0,
	"distCoeffs": [],

	"// RESOLUTION INFORMATION": "",
	"// width and height are the width and height of the input images/video": "",
	"width": 640,
	"height": 480,
	"// screenWidth and screenHeight are the width and height of the display window": "",
	"screenWidth": 1280,
	"screenHeight": 960,

	"// AR VIEW OBJECTS": "[[x, y, z, rx, ry, rz, scale], [x, y, z, rx, ry, rz, scale], ...]",
	"cubes": [[ 0, 4, 12, 53, 10, 250, 2 ], [ -12, 0, 22, 250, 70, 12, 2 ], [ -21, -3, 29, 20, 100, 300, 2 ]]
}
```
Example configuration files for TUM-freiburg1, TUM-freiburg2, TUM-freiburg3, and custom MP4 files are provided in `TUM1Config.json`, `TUM2Config.json`, `TUM3Config.json`, and `mp4Config.json`, respectively. Note, that the camera parameters will need to be changed to match the camera used in your MP4 sequence. The camera parameters currently populating `mp4Config.json` correspond to a Samsung Galaxy S8 smartphone using the Open Camera app.

Additionally, comprehensive usage of all configuration parameters are shown in `mp4Config.json`.

To get started, download the `freiburg3_structure_texture_far` sequence from the [TUM dataset website](https://vision.in.tum.de/data/datasets/rgbd-dataset), update the `inputDataPath` parameter in `TUM3Config.json` to point to its base directory, and run the system with the `-config TUM3Config.json` argument.

**NOTE**: If you find that the system is struggling to initialize a map, set `"smartInitialization"` to `false` to use brute-force initialization. Smart initialization is a proof-of-concept technique whose models were only trained on TUM sequences.

### Map Navigation
The camera for the map view can also be transformed in realtime to analyze the system's map. The controls are as follows:

`W` - move forward\
`S` - move backward\
`A` - strafe left\
`D` - strafe right\
`SPACE` - move up\
`SHIFT` - move down

`←` - turn left\
`→` - turn right\
`↑` - look up\
`↓` - look down

`<` - tilt counter-clockwise\
`>` - tilt clockwise

`J` - decrease move/turn/tilt speed\
`K` - increase move/turn/tilt speed



## Disclaimer
This project is a culmination of the years of research I have done for my Ph.D.; as such, it is research-quality code whose design and coding practices have changed many times over several years. Though the code is annotated and should not be excessively difficult for researchers and other interested parties to parse, I make no claims that this codebase is the pinnacle of good software design.

