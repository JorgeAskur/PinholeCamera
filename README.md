# Position Estimation of recognized objects using YOLOv7 and the Stereo Camera Model.

In this project we explain the development, implementation and integration of a Position Estimation system of recognized objects using YOLOv7. This project is divided into 2 parts, Object Recognition using the artificial vision algorithm YOLOv7 and Pose Estimation based on a Pinhole Camera Model using a ZED 2i camera.

## Authors
- Jorge Askur Vazquez Fernandez
- Jose Miguel Flores Gonzalez

## Dependencies

- Ubuntu 20.04
- [YOLOv7 ROS](https://github.com/lukazso/yolov7-ros)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation)
- [ZED SDK](https://www.stereolabs.com/developers/release/)
- [ZED ROS wrapper](https://github.com/stereolabs/zed-ros-wrapper)

The rest of the required libraries are:

```
  pip install statistics
  pip install opencv-python
  pip install opencv-contrib-python
  sudo apt-get install ros-noetic-image-geometry
  sudo apt-get install ros-noetic-sensor-msgs
  sudo apt-get install ros-noetic-cv-bridge
  sudo apt-get install ros-noetic-visualization-msgs
  sudo apt-get install python3-catkin-tools
```

## Install
```
  git clone https://github.com/JorgeAskur/PinholeCamera.git
  catkin build
```

## Usage

```
#Launch ZED wrapper
roslaunch zed_wrapper zed2i.launch

#In another terminal
#Launch YOLO and Position Estimation
roslaunch yolov7_ros yolov7.launch
```

## Additional Documentation
[Link to technical report](https://drive.google.com/file/d/1Qgoqi3Pur80hLZ6UWWdBX_CbXNdJ_GFX/view?usp=sharing)

[Link to video](https://www.youtube.com/watch?v=ws605s8jNAI)

