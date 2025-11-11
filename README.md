<div align="center">
<h1 align="center">ROS (WIP)</h1>
</div>

<div align="center">

[![License: MIT](https://badgen.net/badge/license/MIT/blue)](LICENSE)
[![Python Version](https://badgen.net/badge/python/3.8/green)](https://www.python.org/)
[![Ubuntu Version](https://badgen.net/badge/Ubuntu/20.04/orange)](https://releases.ubuntu.com/focal/)
[![ROS Version](https://badgen.net/badge/ROS/Noetic/green)](http://wiki.ros.org/noetic)

</div>

<details>

<summary>

## 🚀 Setup

Follow these instructions to set up the ROS environment and run the real-time YOLO detection node.

</summary>

### ⚙️ Requirements

* Ubuntu 20.04
* ROS Noetic
* Python 3.8

#### 🐍 Python Libraries

* `ultralytics`
* `opencv-python`
* `ffmpeg`

### 1. 📦 Workspace Setup (ROS)

First, we need to build the catkin workspace with the required ROS packages.

1. Create your catkin workspace folder structure:

`mkdir -p catkin_ws/src/`

2. Clone this repo:

`git clone https://github.com/frvgmxntx/ROS`

3. Move the `kr_autonomous_flight` folder into your `src` folder:

`mv kr_autonomous_flight catkin_ws/src/`

4. Add the MRS System repository:

`curl https://ctu-mrs.github.io/ppa-stable/add_ppa.sh | bash`

5. Install the MRS System:

`apt install ros-noetic-mrs-uav-system-full`

6. Go to the workspace root `catkin_ws` and configure the build:
```
cd catkin_ws
catkin config --cmake-args -DCMAKE_BUILD_TYPE=releases
```

7. Compile the workspace:

`catkin build`

### 2. 🛠️ Prepare the Detection Script

1. Make the detection script `ROS/detection_ros.py` executable:

`chmod +x detection_ros.py`

Note: Edit the configuration header inside the script so all paths are correct.

### 3. 🏁 Running

1. Open a terminal and source the workspace setup file:

`source catkin_ws/devel/setup.bash`

Note: There are other setup files for diffent shells.

2. Launch the full simulation:

`roslaunch gazebo_utils full_sim.launch`

This should open Gazebo, RViz and a `rqt` command window.

3. On a new terminal, source the workspace setup again then run the inference script on the `scripts` folder:

`python3 detection_ros.py`

The `detection_ros.py` script will start processing the camera topic and record the result. To stop, simply press `Ctrl+c` on the terminal.
After running, the recording video file will be saved in the folder the script was executed.

Note: For now, the codec used `mp4v` may not be compatible with some apps for sharing, to fix this you can reencode using `ffmpeg`:
`ffmpeg -i RESULT_VIDEO_FILE.mp4 -c:v libx264 -crf 23 -an REENCODED_VIDEO_FILE.mp4`

</details>
<details>

<summary>

## 📋 Scripts

Helper scripts for various uses. Check the `scripts` folder.

</summary>

### detection_ros.py

Main script to enable real-time object detection as a ROS node using yolo and opencv (cv_bridge).

It will publish the real-time detection to the image topic `yolo/feed`, you can use ROS `image_view` to connect to it.

The `kr_autonomous_flight` `full_sim.launch` scenario on gazebo will lauch a `rqt` window that has a image_view section to observe the `quadrotor` image topics. The following topics are exposed:

- `/quadrotor/bonus_left/left`

Left side of drone, left cam.

- `/quadrotor/bonus_left/right`

Left side of drone, right cam.

- `/quadrotor/bonus_right/left`

Right side of drone, left cam.

- `/quadrotor/bonus_right/right`

Right side of drone, right cam.

- `/quadrotor/ovc3/left`

Front side of drone, left cam.

- `/quadrotor/ovc3/right`

Front side of drone, right cam.

- `/quadrotor/ovc3`

Front side of drone, both cams.

Those topics publish specific types:

- `image_raw`

Image as seem by cam, no color and uncompressed.

- `image_raw/compressed`

Same as above, less quality but less data stream use.

- `image_raw/compressedDepth`

Depth image (float32 pixels representing depth), compressed as above.

- `image_raw/theora`

Compressed image as a video stream, using the `theora` codec.

### split_dataset.py

Split any video into frames, then create the yolo dataset structure with train, validation and test folders. (WIP)

</details>

## 🤖 YOLO

Instructions on how to fine-tune a model are on `train/README.md`

<details> 
  
<summary>
<h2> 🎬 Showcase </h2>
</summary>

Check `showcase` folder.

</details>

## 🪜 Roadmap

Planned features:
- [x] Implement real-time YOLO detection on a ROS topic.
- [x] Show feed from image topic while running.
- [x] List usable camera sensors from `quadrotor` model. See `Scripts` section.
- [x] Allow passing model as a parameter.
- [ ] Generate report after running.

## 📚 Acknowledgments and Citations

This project use code and concepts from other research work. If you use this code in your work, please cite as well:

<details>

<summary><h4><b>UAV Navigation in Dense Forest</b></h4></summary>

[github](https://github.com/sebnem-byte/UAV-Navigation-in-Dense-Forest/tree/main)

</details>

<details>

<summary><h4><b>Forest Generator for Gazebo</b></h4></summary>

[github](https://github.com/hurkansah/forest_gen)

</details>

<details>

<summary><h4 align="center"><b>Fast, autonomous flight in GPS-denied and cluttered environments</b></h4></summary>

[github](https://github.com/KumarRobotics/kr_autonomous_flight)

```
@article{mohta2018experiments,
  title={Experiments in fast, autonomous, gps-denied quadrotor flight},
  author={Mohta, Kartik and Sun, Ke and Liu, Sikang and Watterson, Michael and Pfrommer, Bernd and Svacha, James and Mulgaonkar, Yash and Taylor, Camillo Jose and Kumar, Vijay},
  booktitle={2018 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={7832--7839},
  year={2018},
  organization={IEEE}
}
```

```
@article{mohta2018experiments,
  title={Experiments in fast, autonomous, gps-denied quadrotor flight},
  author={Mohta, Kartik and Sun, Ke and Liu, Sikang and Watterson, Michael and Pfrommer, Bernd and Svacha, James and Mulgaonkar, Yash and Taylor, Camillo Jose and Kumar, Vijay},
  booktitle={2018 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={7832--7839},
  year={2018},
  organization={IEEE}
}
```

```
@article{liu2022large,
  title={Large-Scale Autonomous Flight With Real-Time Semantic SLAM Under Dense Forest Canopy},
  author={Liu, Xu and Nardari, Guilherme V. and Ojeda, Fernando Cladera and Tao, Yuezhan and Zhou, Alex and Donnelly, Thomas and Qu, Chao and Chen, Steven W. and Romero, Roseli A. F. and Taylor, Camillo J. and Kumar, Vijay},
  journal={IEEE Robotics and Automation Letters},
  year={2022},
  volume={7},
  number={2},
  pages={5512-5519},
}
```

</details>
