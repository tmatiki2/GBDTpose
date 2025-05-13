# **GBDTpose**

**GBDTpose** is a vision-based parametric localization framework. This repository contains starter code to help you get started.  
Example videos demonstrate drone localization in action.

📹 **Watch the demo videos here:**  
[Google Drive - GBDTpose Demos](https://drive.google.com/drive/folders/126kFGxMAw3pU-g0phBOq0ywAM3b9NQak)
---

## **Data**

The repository includes:

- **Template images** and **target images** — feel free to replace them with your own.
- **Template poses** (ground truth) and **target poses** — available in the respective directories.

---

## **Simulation with Gazebo**

To stream images from the GBDT in real-time using Gazebo:

Download **Gazebo v9** here:  
[https://classic.gazebosim.org/install](https://classic.gazebosim.org/install)  
Be sure to select **version 9**, which is compatible with ROS Noetic.

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
A template and target image folder is provided containing GBDT images of example strcuture used in the paper.
Replace these images with Yours

## **5. Download the GBDTpose deployment notebook and the EKF python script**
The deploymement notebook uses the customized model to predict poses given input images, and publishes the pose to the UAV.
If the T265 camera is used, run the GBDTpose EKF script for integration. Mavros is required for transmitting the poses to the UAV. 
Follow these instrcutions to setup mavros for Ardupilot flight controller.

