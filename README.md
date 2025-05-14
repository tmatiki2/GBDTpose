# **GBDTpose**

**GBDTpose** is a vision-based parametric localization framework. This repository contains starter code to help you get started.  
Example videos demonstrate drone localization in action.

📹 **Watch the demo videos here:**  
[Google Drive - GBDTpose Demos](https://drive.google.com/drive/folders/126kFGxMAw3pU-g0phBOq0ywAM3b9NQak)
---

## **Data**

The repository includes:

- **Template images** and **source images** — feel free to replace them with your own.
- **Template poses** and **source poses** — available in the respective directories.

---

## **Simulation with Gazebo**

To stream images from the GBDT in real-time using Gazebo:

Download **Gazebo v9** here:  
[https://classic.gazebosim.org/install](https://classic.gazebosim.org/install)  
Be sure to select **version 9**, which is compatible with ROS Melodic.
To create and launch the world file for Gazebo, follow this link:
[https://classic.gazebosim.org/tutorials?tut=ros_roslaunch](https://classic.gazebosim.org/tutorials?tut=ros_roslaunch)

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
