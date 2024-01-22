#### Launch the simulation


 Launch the ur10e commander file:
  ```shell
  roslaunch commander ur10e_ka_commander.launch
  ```

 Go to file **commander/notebooks/commander_ak_examples.ipynb** and run the code snippets to see how MoveIt works.

 
#### Launch the real robot

Set:

- Robot IP: 192.168.56.101
- Your laptop IP: 192.168.56.1

Launch the robot bringup, this file sets the robot IP and loads the kinematics calibration for the IAAC UR10e.
 ```shell
  roslaunch commander ur10e_ka_commander.launch sim:=false
 ```

If you want to use the gripper instead, replace ka by gripper.

To capture and reconstruct everything sensed with the Azure Kinect use **capture_manager/notebooks/reconstruction.ipynb** as reference

edit dockerfile
Installation (docker) + azure kinetic
Docker run (normal and nvidia)
Usage -> explain conection with gh_ros repository
-change ip address in launch file

add all .ply to captures