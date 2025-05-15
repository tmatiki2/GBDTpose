# **Parametric GBDTpose**

**Parametric GBDTpose** is a vision-based localization framework that leverages a Graphics-Based Digital Twin (GBDT) to train a siamese neural network for accurate pose estimation. The GBDT being a photorealistic digital replica of the physical environment, provides synthetic image data for training and evaluating the pose regressor. This pose regressor (GBDTpose) can be integrated with the Pixhawk flight controller, Ardupilot, and Mission Planner software for UAV navigation in GPS-denied environments.
Example videos demonstrate drone localization in action using GBDTpose. 

📹 **Watch the demo videos here:**  
[Google Drive - GBDTpose Demos](https://drive.google.com/drive/folders/126kFGxMAw3pU-g0phBOq0ywAM3b9NQak)

This repository contains starter code to help you get started with training and deploying **GBDTpose** on your custom UAV platform.
If you find this repository useful, cite the paper: 
"*Leveraging Graphics-Based Digital Twin to Develop Scalable Parametric Pose Regressor for UAV Localization in GPS-denied Environments*".
Contact the author at (thomasngare5@gmail.com).

## **Data**

The repository includes:

- **Template images** and **source images** — feel free to replace them with your own.
- **Template poses** and **source poses** — available in the respective directories.

---

## **Simulation with Gazebo**

To fine-tune/train GBDTpose, still images of the GBDT in **Blender** can be rendered at predetermined poses; however, this can be time-consuming.
Alternatively, Gazebo can be used to stream images of the GBDT in real-time for training the Siamese network as presented in the paper. 
Follow instructions below to setup Gazebo. Note that Ubuntu 18.04 and ROS Melodic is required.

Download **Gazebo v9** here:  
[https://classic.gazebosim.org/install](https://classic.gazebosim.org/install)

Be sure to select **version 9**, which is compatible with ROS Melodic.

An example Gazebo world file **example_world.world** that hosts the GBDT together with fisheye camera sensors is provided in the **gbdt_package**. Replace the **sdf** file path (line ##) with your custom GBDT sdf model. A python code **agent.py** to reconfigure the fisheye cameras with ROS is provided in **gbdt_package**, together with the launch file **agent.launch**. Downlaod the **gbdt_package** to your **catkin/src** workspace, then run the following:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch gbdt_package agent.launch
```
More details on use of Gazebo and ROS for image streaming can be found in the following link:
[https://classic.gazebosim.org/tutorials?tut=ros_roslaunch](https://classic.gazebosim.org/tutorials?tut=ros_roslaunch)
[https://classic.gazebosim.org/tutorials/?tut=ros_comm](https://classic.gazebosim.org/tutorials/?tut=ros_comm)

It is recommended to first stream and store template images from the fisheye cameras in a folder and recall them during training, as presented in the paper. Run the **streaming_images.ipynd** notebook to subscripe to images of the GBDT from the fisheye cameras in Gazebo.

---

## **Installation Instructions**

### **1. Install the Required Python Packages**

It's recommended to use a virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## **2. Install ROS (for UAV Deployment)**

**GBDTpose** uses **ROS Melodic** for UAV integration (tested on **Ubuntu 18.04**).  
To install ROS Melodic, run:

```bash
sudo apt update
sudo apt install ros-melodic-desktop-full
```
## **3. Download pretarined weights from the releases -- For Resnet and VIT backbones**
This provides pretrained weights that can be used to initialize fine-tuning of a custom model

## **4. Download the training notebook and provide directories to your template and target images**
A template and target image folder is provided containing GBDT images and ground truth poses of example structure used in the paper.
Replace these images with yours. Alternatively, Gazebo can be used to stream images at various poses in real-time for training.

## **5. Download the Deployment Notebook and EKF Script**

The **GBDTpose deployment notebook** uses the custom-trained model to predict poses from input images and publish them to the UAV.  

If you're using an Intel RealSense **T265** camera, the **GBDTpose EKF script**, which fuses visual odometry with predicted poses using an Extended Kalman Filter (EKF) can be used to improve localization.

To transmit poses to your UAV, **MAVROS** is required. MAVROS acts as a communication bridge between ROS and the flight controller (e.g., Pixhawk with ArduPilot or PX4).

---

### **MAVROS Installation for ArduPilot/PX4**

Follow the official guide to install MAVROS and set it up with your flight controller:  
🔗 [MAVROS Installation Guide – ArduPilot/PX4](https://ardupilot.org/dev/docs/ros-install.html)

---

### **Launching MAVROS**

To connect MAVROS to your flight controller over USB or telemetry (e.g., `/dev/ttyUSB0`), use:

```bash
roslaunch mavros apm.launch fcu_url:=/dev/ttyUSB0:57600
