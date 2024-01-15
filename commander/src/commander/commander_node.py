from typing import List, Optional, Tuple, Union

import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from moveit_msgs.msg import (
    DisplayTrajectory,
    MotionPlanRequest,
    MotionPlanResponse,
    MotionSequenceItem,
    MoveGroupSequenceAction,
    MoveItErrorCodes,
)
from moveit_msgs.srv import (
    GetMotionPlan,
    GetMotionPlanRequest,
    GetMotionPlanResponse,
    GetMotionSequence,
    GetMotionSequenceRequest,
    GetMotionSequenceResponse,
)
from trajectory_msgs.msg import JointTrajectoryPoint

from commander.msg import Goal
from commander.srv import (
    ExecuteTrajectory,
    GetTcpPose,
    GetTcpPoseResponse,
    PickPlace,
    PickPlaceRequest,
    PlanGoal,
    PlanGoalRequest,
    PlanGoalResponse,
    PlanSequence,
    PlanSequenceRequest,
    PlanSequenceResponse,
    SetEe,
    TraceTrajectory,
)
from commander.ur_robot import UrRobot


def error_code_to_str(error_code_val):
    code_dict = {}

    # Access the class attributes of MoveItErrorCodes and store in dictionary
    for attr, value in MoveItErrorCodes.__dict__.items():
        # Filter out the special attributes and lists
        if not attr.startswith('__') and not isinstance(value, list):
            code_dict[value] = attr

    return code_dict.get(error_code_val, 'UNKNOWN_ERROR_CODE')


class CommanderNode:
    JOINT_ERROR_THRESHOLD = 0.001
    JOINT_MEAN_ERROR_THRESHOLD = 0.0005
    MAX_PLANNING_ATTEMPTS = 5
    DO_PIN = 0

    def __init__(self, planning_group: str = 'manipulator') -> None:
        rospy.init_node('commander_node', anonymous=True)
        self.move_group = moveit_commander.MoveGroupCommander(planning_group)
        self.move_group.set_num_planning_attempts(10)
        self.robot = moveit_commander.RobotCommander()
        self.robot_links = self.robot.get_link_names()
        self.sim = rospy.get_param('/commander/sim', True)

        self._init_services()
        self._init_service_proxies()

        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', DisplayTrajectory, queue_size=20
        )

        self.trajectory = None

        if not self.sim:
            self.robot_interface = UrRobot()
            self.set_payload(3.0, Vector3(0.0, 0.0, 0.0))

    def _init_service_proxies(self):
        rospy.wait_for_service('/plan_kinematic_path', timeout=10)
        self.get_motion_sequence_plan_srv = rospy.ServiceProxy(
            '/plan_sequence_path', GetMotionSequence
        )

        rospy.wait_for_service('/plan_sequence_path', timeout=10)
        self.get_motion_plan_srv = rospy.ServiceProxy('/plan_kinematic_path', GetMotionPlan)

        rospy.wait_for_service('/trace_trajectory', timeout=10)
        self.trace_trajectory_service = rospy.ServiceProxy('/trace_trajectory', TraceTrajectory)

    def _init_services(self):
        self.plan_goal = rospy.Service('/commander/plan_goal', PlanGoal, self.plan_goal_cb)
        self.plan_sequence = rospy.Service(
            '/commander/plan_sequence', PlanSequence, self.plan_sequence_cb
        )
        self.execute_trajectory = rospy.Service(
            '/commander/execute_trajectory', ExecuteTrajectory, self.execute_trajectory_cb
        )
        self.get_tcp_pose = rospy.Service(
            '/commander/get_tcp_pose', GetTcpPose, self.get_tcp_pose_cb
        )
        self.set_ee = rospy.Service('/commander/set_ee', SetEe, self.set_ee_cb)
        self.pick_place = rospy.Service('/commander/pick_place', PickPlace, self.pick_place_cb)

    def run(self):
        rospy.loginfo(f'commander: running {"in simulation" if self.sim else "on robot"} mode')
        while not rospy.is_shutdown():
            rospy.spin()

    def plan_goal_cb(self, req: PlanGoalRequest) -> PlanGoalResponse:
        self.trajectory = None
        error_code, trajectory = self.get_motion_plan(req.goal)
        if error_code == MoveItErrorCodes.NO_IK_SOLUTION:
            error_code, trajectory = self.retry_planning(req.goal)
        if error_code == MoveItErrorCodes.SUCCESS:
            configuration_change = self.check_joint_configuration_change(
                trajectory.joint_trajectory.points[0].positions,
                trajectory.joint_trajectory.points[-1].positions,
                tolerance=0.1,
            )
            rospy.loginfo(
                f'commander: successfully planned goal, configuration change: {configuration_change}'
            )
            self.trajectory = trajectory
            self.trace_trajectory_service(trajectory, self.move_group.get_end_effector_link())
            return PlanGoalResponse(success=True, configuration_change=configuration_change)
        rospy.loginfo(f'commander: failed to plan goal: {error_code_to_str(error_code)}')

        return PlanGoalResponse(success=False)

    def retry_planning(
        self, goal: Goal
    ) -> Tuple[int, Optional[Union[MoveItErrorCodes, MotionPlanResponse]]]:
        for attempt in range(self.MAX_PLANNING_ATTEMPTS):
            rospy.loginfo(f'commander: no ik solution found, replanning attempt {attempt + 1}')
            error_code, trajectory = self.get_motion_plan(goal)

            if error_code == MoveItErrorCodes.SUCCESS:
                break
        return error_code, trajectory

    def plan_sequence_cb(self, req: PlanSequenceRequest) -> PlanSequenceResponse:
        self.trajectory = None
        error_code, trajectory = self.get_motion_sequence_plan(req.goals, req.blends)
        if (len(trajectory)) > 1:
            rospy.logerr('commander: multiple trajectories !!!!!!!')
        if error_code == MoveItErrorCodes.NO_IK_SOLUTION:
            error_code, trajectory = self.retry_planning(req.goal)
        if error_code == MoveItErrorCodes.SUCCESS:
            configuration_change = self.check_joint_configuration_change(
                trajectory[0].joint_trajectory.points[0].positions,
                trajectory[0].joint_trajectory.points[-1].positions,
                tolerance=0.1,
            )

            rospy.loginfo(
                f'commander: successfully planned sequence, configuration change: {configuration_change}'
            )
            self.trajectory = trajectory[0]
            self.display_trajectory_publisher.publish(DisplayTrajectory(trajectory=[trajectory[0]]))
            self.trace_trajectory_service(trajectory[0], self.move_group.get_end_effector_link())
            return PlanSequenceResponse(success=True, configuration_change=configuration_change)
        rospy.loginfo(f'commander: failed to plan sequence: {error_code_to_str(error_code)}')
        return PlanSequenceResponse(False)

    def execute_trajectory_cb(self, req: ExecuteTrajectory) -> bool:
        if self.trajectory is None:
            rospy.logerr('commander: no trajectory to execute')
            return False
        self.move_group.execute(self.trajectory, wait=True)
        if not self.check_goal_reached(self.trajectory.joint_trajectory.points[-1].positions):
            rospy.logerr('commander: goal not reached')
            raise RuntimeError('Goal not reached')
            # return False
        self.trajectory = None
        return True

    def get_tcp_pose_cb(self, req: GetTcpPose) -> GetTcpPoseResponse:
        return GetTcpPoseResponse(self.get_current_pose())

    def set_ee_cb(self, req: SetEe) -> bool:
        if req.endeffector_link not in self.robot_links:
            rospy.logerr(f'commander: invalid ee link: {req.endeffector_link}')
            return False
        self.move_group.set_end_effector_link(req.endeffector_link)
        if self.move_group.get_end_effector_link() == req.endeffector_link:
            rospy.loginfo(f'commander: end effector link set to {req.endeffector_link}')
            return True
        return False

    def pick_place_cb(self, req: PickPlaceRequest) -> bool:
        success_do = True
        success_at = True
        if not self.sim:
            success_do = (
                self.robot_interface.set_do(self.DO_PIN, 1)
                if req.state
                else self.robot_interface.set_do(self.DO_PIN, 0),
            )
        if req.co_id is not None:
            success_at = (
                self.move_group.attach_object(req.co_id)
                if req.state
                else self.move_group.detach_object(req.co_id)
            )

        return success_do and success_at

    def get_current_pose(self) -> Pose:
        self.move_group.stop()
        return self.move_group.get_current_pose(self.move_group.get_end_effector_link()).pose

    def set_payload(self, mass: float, cog: Vector3):
        success = self.robot_interface.set_payload(mass, cog)
        if success:
            rospy.loginfo(f'commander: set payload to {mass} kg')
        else:
            rospy.logerr('commander: failed to set payload')

    def set_up_move_group(self, vel: float, acc: float, planner: str):
        self.move_group.clear_pose_targets()
        self.move_group.set_max_velocity_scaling_factor(vel)
        self.move_group.set_max_acceleration_scaling_factor(acc)
        if planner == 'ompl':
            self.move_group.set_planning_pipeline_id('ompl')
            self.move_group.set_planner_id('RRTConnect')
            self.move_group.set_goal_joint_tolerance(0.001)
        elif planner == 'ptp':
            self.move_group.set_planning_pipeline_id('pilz_industrial_motion_planner')
            self.move_group.set_planner_id('PTP')
        elif planner == 'lin':
            self.move_group.set_planning_pipeline_id('pilz_industrial_motion_planner')
            self.move_group.set_planner_id('LIN')
        else:
            raise ValueError(f'Invalid planner: {planner}')

    def get_motion_plan_req(self, goal: PlanGoal) -> GetMotionPlanRequest:
        self.set_up_move_group(goal.vel_scale, goal.acc_scale, goal.planner)

        if goal.pose != Pose():  # check if msg is initialized, if all fields are zero
            rospy.loginfo('commander: planning pose goal')
            self.move_group.set_pose_target(goal.pose)
        elif len(goal.joint_values) > 0:
            rospy.loginfo('commander: planning joint goal')
            self.move_group.set_joint_value_target(goal.joint_values)

        return self.move_group.construct_motion_plan_request()

    def get_motion_plan(self, goal: Goal) -> GetMotionPlanResponse:
        req = self.get_motion_plan_req(goal)
        resp = self.get_motion_plan_srv(req)
        return resp.motion_plan_response.error_code.val, resp.motion_plan_response.trajectory

    def get_motion_sequence_plan(
        self, goals: List[Goal], blends: List[float]
    ) -> GetMotionSequenceResponse:
        msr = GetMotionSequenceRequest()
        for i in range(len(goals)):
            msi = MotionSequenceItem()
            msi.req = self.get_motion_plan_req(goals[i])
            msi.blend_radius = blends[i] if i < len(blends) and blends[i] is not None else 0.0
            msr.request.items.append(msi)
        resp = self.get_motion_sequence_plan_srv(msr)
        return resp.response.error_code.val, resp.response.planned_trajectories

    # def check_joint_configuration_change(
    #     self,
    #     start: JointTrajectoryPoint.positions,
    #     end: JointTrajectoryPoint.positions,
    #     tolerance=0.1,
    # ):
    #     signs = np.sign(np.array(start) * np.array(end))

    #     joint_changes_small = True

    #     for i, sign in enumerate(signs):
    #         if sign < 0:
    #             if abs(start[i]) < tolerance or abs(end[i]) < tolerance:
    #                 rospy.logdebug('Joint changes sign, but the change is small. Ignoring.')
    #                 rospy.logdebug(f'start[i] = {start[i]}, end[i] = {end[i]}')
    #                 continue
    #             rospy.logwarn(f'Joint angle {str(i)} would change sign!')
    #             print(f'start[i] = {start[i]}, end[i] = {end[i]}')
    #             joint_changes_small = False

    #     return not joint_changes_small

    def check_joint_configuration_change(
        self,
        start: JointTrajectoryPoint.positions,
        end: JointTrajectoryPoint.positions,
        tolerance=0.1,
        ignore_joint_0=True,
    ):
        signs = np.sign(np.array(start) * np.array(end))

        joint_changes_small = True

        for i, sign in enumerate(signs):
            if i == 0 and ignore_joint_0:
                continue
            if sign < 0:
                if abs(start[i]) < tolerance or abs(end[i]) < tolerance:
                    rospy.logdebug('Joint changes sign, but the change is small. Ignoring.')
                    rospy.logdebug(f'start[{i}] = {start[i]}, end[{i}] = {end[i]}')
                    continue
                rospy.logwarn(f'Joint angle {i} would change sign!')
                print(f'start[{i}] = {start[i]}, end[{i}] = {end[i]}')
                joint_changes_small = False

        return not joint_changes_small

    def check_goal_reached(self, target_joint_values: List[float]) -> bool:
        distance = [
            abs(target - current)
            for target, current in zip(
                target_joint_values, self.move_group.get_current_joint_values()
            )
        ]

        if [i for i in distance if i >= self.JOINT_ERROR_THRESHOLD]:
            return False

        return sum(distance) / len(target_joint_values) < self.JOINT_MEAN_ERROR_THRESHOLD


if __name__ == '__main__':
    commander = CommanderNode()
    commander.run()
