## Intelligent Vehicle ROS Project 🚗🤖
✅ Real-time SLAM mapping & localization
✅ Global / Local path planning
✅ YOLO-based object detection
✅ Speech interaction (ASR + TTS)
✅ Autonomous navigation with obstacle avoidance

## Introduction
This project is developed based on the **ROS (Robot Operating System)** framework for the **National College Student Intelligent Vehicle Competition**.  
By integrating multiple sensors including **LiDAR, RGB camera, IMU, and microphone array**, the system achieves:

- **SLAM Mapping & Localization**: Real-time map building with AMCL and EKF for improved accuracy.  
- **Path Planning & Navigation**: Supports global planning (Dijkstra / A*) and local planning (DWA / TEB) with obstacle avoidance.  
- **Object Detection**: YOLO-based deep learning model for human detection and recognition.  
- **Speech Recognition & Synthesis**: RNN/Deep Neural Network for voice command recognition and TTS for voice output.  
- **Motion Control**: Controller manager for wheel velocity and steering control.  

This framework can be applied to **autonomous cars, indoor mobile robots, and service robots**.

## Project Structure

The project is organized as follows (located under `~/Smart_car/src/`):

- **geometry/** – Geometric utilities  
- **geometry2/** – Extended geometric utilities  
- **iris_lama/** – SLAM algorithm (LiDAR-based mapping)  
- **logic_moudle/** – Decision-making and logic control  
- **robot_localization/** – Sensor fusion & localization (EKF/UKF)  
- **ucar_cam/** – Camera driver & image processing  
- **ucar_controller/** – Motion control for the vehicle  
- **ucar_map/** – Map management and storage  
- **ucar_nav/** – Path planning & navigation  
- **xf_mic_asr_offline/** – Offline speech recognition (ASR)  
- **ydlidar/** – LiDAR driver (YDLIDAR series)  
- **yolov4-for-darknet_ros/** – YOLOv4 object detection integrated with ROS
---

## System Architecture
- **Hardware**: LiDAR, RGB camera, IMU, microphone array, embedded computing platform (Ubuntu 18.04).  
- **Software**: ROS Melodic, Gazebo, RViz, Deep Learning Frameworks (TensorRT/YOLO).  
- **ROS Packages**:
  - `ucar_controller`: Motion control  
  - `ucar_nav`: Path planning & navigation  
  - `slam_gmapping`: Mapping  
  - `speech_recognition`: Speech recognition & TTS  
  - `vision_yolo`: Object detection  

---

## Environment Setup

### 1. System Requirements
- Ubuntu 18.04  
- ROS Melodic  
- CUDA / cuDNN (optional, for deep learning acceleration)  

### 2. Install Dependencies
```bash
sudo apt-get update
sudo apt-get install ros-melodic-desktop-full
sudo apt-get install ros-melodic-navigation ros-melodic-slam-gmapping
sudo apt-get install python-rosdep python-rosinstall

### 3. Initialize Workspace
```bash
cd ~/Smart_car/
catkin_make
source devel/setup.bash

## Usage

```bash
# Launch SLAM Mapping
roslaunch ucar_nav slam_gmapping.launch
# ➡️ Drive the vehicle around to collect LiDAR data and generate the environment map.

# Localization & Navigation
roslaunch ucar_nav navigation.launch
# ➡️ Open RViz, set a goal, and the vehicle will automatically plan and follow the path.

# Speech Recognition & TTS
rosrun speech_recognition asr_node.py
rosrun speech_recognition tts_node.py
# ➡️ Enable voice command recognition and text-to-speech feedback.

# Object Detection (YOLO)
roslaunch vision_yolo yolo_detect.launch
# ➡️ Detect humans or objects on the track and trigger corresponding actions.
