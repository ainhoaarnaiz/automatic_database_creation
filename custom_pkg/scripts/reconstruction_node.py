#!/usr/bin/env python3

from typing import List, Tuple
from math import pi
from datetime import datetime
import rospy
from copy import deepcopy
import os
import yaml

from moveit_commander import PlanningSceneInterface

from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Point,
    Quaternion,
    Vector3,
)
from commander.msg import Goal
from commander.srv import (
    ExecuteTrajectory,
    PlanGoal,
    PlanGoalRequest,
    PlanSequence,
    PlanSequenceRequest,
    PickPlace,
    GetTcpPose,
    VisualizePoses,
    SetEe,
)
from industrial_reconstruction_msgs.srv import (
    StartReconstruction,
    StartReconstructionRequest,
    StopReconstruction,
    StopReconstructionRequest,
)

from commander.utils import poses_from_yaml, load_scene

CAPTURE = True

rospy.init_node("reconstruction")

ply_file_path = rospy.get_param("~ply_file_path", "/dev_ws/src/software_II_project/custom_pkg/captures/raw_01.ply")

load_scene()

plan_goal_srv = rospy.ServiceProxy("commander/plan_goal", PlanGoal)
plan_sequence_srv = rospy.ServiceProxy("commander/plan_sequence", PlanSequence)
execute_trajectory_srv = rospy.ServiceProxy("commander/execute_trajectory", ExecuteTrajectory)
get_tcp_pose_srv = rospy.ServiceProxy("commander/get_tcp_pose", GetTcpPose)
set_ee_srv = rospy.ServiceProxy("commander/set_ee", SetEe)
pick_place_srv = rospy.ServiceProxy("commander/pick_place", PickPlace)

if CAPTURE:
    start_recon = rospy.ServiceProxy("/start_reconstruction", StartReconstruction)
    stop_recon = rospy.ServiceProxy("/stop_reconstruction", StopReconstruction)


def display_poses(poses: List[Pose], frame_id: str = "base_link") -> None:
    rospy.wait_for_service("/visualize_poses", timeout=10)
    visualize_poses = rospy.ServiceProxy("/visualize_poses", VisualizePoses)
    visualize_poses(frame_id, poses)


def gen_recon_msg(path: str) -> Tuple[StartReconstructionRequest, StopReconstructionRequest]:
    start_srv_req = StartReconstructionRequest()
    start_srv_req.tracking_frame = "rgb_camera_link"
    start_srv_req.relative_frame = "base_link"
    start_srv_req.translation_distance = 0.0
    start_srv_req.rotational_distance = 0.0
    start_srv_req.live = True
    start_srv_req.tsdf_params.voxel_length = 0.003 ##########last value 0.02
    start_srv_req.tsdf_params.sdf_trunc = 0.01 ###########last value 0.04
    start_srv_req.tsdf_params.min_box_values = Vector3(x=0.0, y=0.0, z=0.0)
    start_srv_req.tsdf_params.max_box_values = Vector3(x=0.0, y=0.0, z=0.0)
    start_srv_req.rgbd_params.depth_scale = 1000
    start_srv_req.rgbd_params.depth_trunc = 10000
    start_srv_req.rgbd_params.convert_rgb_to_intensity = False

    stop_srv_req = StopReconstructionRequest()
    #path = path + datetime.now().strftime("%m_%d_%H_%M") + ".ply"
    stop_srv_req.mesh_filepath = path

    return start_srv_req, stop_srv_req

cam_home = [-3.062046195720626, -2.042543974749485, -0.9841965193219935, -1.8468536888817202, -1.4911785421602695, -3.2211255818224362]
plan_goal_srv(Goal(joint_values=cam_home, vel_scale=0.2, acc_scale=0.2, planner='ptp'))
success = execute_trajectory_srv()

success = set_ee_srv('rgb_camera_tcp')

def execute_joint_states(joint_position):
    # Plan the goal using the stored joint position
    success = plan_goal_srv(Goal(joint_values=joint_position, vel_scale=0.02, acc_scale=0.02, planner='ptp')).success

    # Check if planning is successful
    if success:
        # Execute the trajectory
        success = execute_trajectory_srv()

        # Check if execution is successful
        if not success:
            rospy.loginfo("Failed to execute trajectory")
            exit()
    else:
        rospy.loginfo("Failed to plan")
        exit()

#current_directory = os.getcwd()

# Read joint positions from the YAML file
#yaml_filename = os.path.join(current_directory, 'joint_positions.yaml')
yaml_filename = "/dev_ws/src/software_II_project/custom_pkg/config/joint_positions.yaml"

try:
    with open(yaml_filename, 'r') as yaml_file:
        joint_positions_data = yaml.safe_load(yaml_file)
        joint_positions = joint_positions_data.get('joint_positions', [])
except FileNotFoundError:
    rospy.loginfo(f"YAML file '{yaml_filename}' not found. Please run the code that saves joint positions first.")
    exit()

if CAPTURE:
    start_recon_req, stop_recon_req = gen_recon_msg(ply_file_path)

start = True

for joint_position in joint_positions:
    
    execute_joint_states(joint_position)

    if start:
        print("Start recon")
        if CAPTURE:
            start_recon(start_recon_req)
            start = False


if CAPTURE:
    stop_recon(stop_recon_req)