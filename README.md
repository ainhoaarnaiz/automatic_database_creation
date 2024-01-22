
# Automatic database creation using ROS - Grasshopper

This repository contains all our developments for the Software II group project. We explore the idea of an automatic 3D-scanned object database creator: 

![Workflow](/z_images/workflow.png)

A person will place an object (i.e. stick) on the robots workspace and this will scan it, this then will be post processed  and sent to rhino/grasshopper to add it to a database, this denoised data will parallely be sent to another ros node that will extract all possible grasping points and choose the most suitable one to put the object aside so that the person can place a new element to scan.

So far, the developments regarding the scanning and Grasshopper - ROS communication have been succesfully completed. However, we plan on keeping exploring and developing the following:

- [x] Scaning and Filtering (using reconstruction and open3d)
- [x] ROS - Grasshopper communication
- [x] PointCloud subscriber GH
- [ ] Trying different scanning approaches - photogrammetry, gaussian splatting…
- [ ] Digital vs. Physical (analyze results accuracy)
- [ ] Mesh database creation in grasshopper
- [ ] Automating scanning toolpath - bounding box strategy
- [ ] Extract grasping poses from object


## Hardware Setup

Fot this example, we will be using an UR10e and a Azure Kinetic camera:

![Hardware Setup](/z_images/robot_diagram.png)

## Installation

#### Azure Kinetic Setup

In a new terminal, install v4l-utils:

  ```shell
  sudo apt install v4l-utils
  ```

Download [this](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules) file and go to Downloads directory in the terminal. Once there, enter:

  ```shell
  mv 99-k4a.rules /etc/udev/rules.d/
  ```
Restart the computer.

#### Build Docker Image

Follow the steps in [here](https://docs.docker.com/engine/install/ubuntu/) and [here](https://docs.docker.com/engine/install/linux-postinstall/) to properly install Docker.

Clone this repository and go to its directory in the terminal. Then:

  ```shell
  .docker/build_image.sh
  ```

Now, if you don't have a Nvidia driver, run this command to create a container:

  ```shell
  .docker/run_user.sh
  ```

If you have a Nvidia driver follow the instructions [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) to install the NVIDIA Container Toolkit and then run:

  ```shell
  .docker/run_user_nvidia.sh
  ```

Once inside the container, ake ownership of the workspace with:


  ```shell
  sudo chown -R $USER /dev_ws
  ```

Open vscode, go to the docker tab. Select the running container, right click and select attach vscode.

**Important note**: STOP and START the container from vscode. If you do .docker/run_user.sh again, you will create a new container, and you will loose all the progress. So, make sure that you start and stop the same container always.

#### Install COMPAS FAB

To install COMPAS FAB, follow the intructions in [gh_ros](https://github.com/ainhoaarnaiz/gh_ros) repository.

## Usage

#### Launch the Simulation

Launch the ur10e commander file:
  ```shell
  roslaunch commander ur10e_ka_commander.launch
  ```

Go to file **custom_pkg/notebooks/commander_ak_examples.ipynb** and run the code snippets to see how MoveIt works. This file also generates the **joint_positions.yaml** file, which is then used to run the **reconstruction_node**.

 
#### Launch the Real Robot

Set:

- Robot IP: 192.168.56.101
- Your laptop IP: 192.168.56.1

Launch the robot bringup, this file sets the robot IP and loads the kinematics calibration for the IAAC UR10e.

 ```shell
  roslaunch commander ur10e_ka_commander.launch sim:=false
 ```

If you want to use the gripper instead, replace ka by gripper.

Launch the Azure Kinect industrial reconstruction node by entering:

 ```shell
  roslaunch capture_manager reconstruction.launch
 ```

Edit the **rosbridge_websocket.launch** file inside the custom_pkg, and change the address argument to your **IP address** (i.e. 172.16.21.74). After setting it up, launch it:

 ```shell
  roslaunch custom_pkg rosbridge_websocket.launch
 ```

Finally, launch the launch file that executes all the scanning and filtering nodes. Change the .ply file name by editing the **file_path** (raw pointcloud) and **save_path** (filteres pointcloud) arguments in the launch file (**pcl_capture.launch**).

 ```shell
  roslaunch custom_pkg pcl_capture.launch
 ```

## Results

You can find a couple of our .ply files inside the custom_pkg/captures folder. Enjoy!


![Results](/z_images/results.png)