# **GBDTpose**

This repository contains starter code for implementing **GBDTpose**, a vision-based pose estimation pipeline.  
Example videos demonstrate drone localization using GBDTpose.

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
