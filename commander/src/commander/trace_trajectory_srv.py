#!/usr/bin/env python3

from typing import List, Optional

import PyKDL
import rospy
from kdl_parser_py.urdf import treeFromUrdfModel
from moveit_commander.conversions import list_to_pose
from std_msgs.msg import ColorRGBA, Header
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker, MarkerArray

from commander.srv import TraceTrajectory

NUM_OFF_JOINTS = 6

MARKER_SCALE = 0.01
START_COLOR = ColorRGBA(1.0, 0.0, 0.0, 0.7)
END_COLOR = ColorRGBA(1.0, 1.0, 0.0, 0.7)


def print_kdl_chain(chain: PyKDL.Chain) -> None:
    chain_str = []
    chain_str.append('KDL chain:')
    for i in range(chain.getNrOfSegments()):
        chain_str.append(f'{i}: {chain.getSegment(i).getName()}')

    return ','.join(chain_str)


def get_kdl_fk_solver(
    base_link: str = 'base_link', tip_link: str = 'tool0'
) -> Optional[PyKDL.ChainFkSolverPos_recursive]:
    urdf = URDF.from_parameter_server()
    success, tree = treeFromUrdfModel(urdf)
    if not success:
        rospy.logerr('trajectory tracer: failed to parse urdf')
        return None
    chain = tree.getChain(base_link, tip_link)
    if chain.getNrOfSegments() == 0:
        rospy.logerr(f'trajectory tracer: failed to get chain from {base_link} to {tip_link}')
        return None
    rospy.loginfo(f'trajectory tracer: {print_kdl_chain(chain)}')

    return PyKDL.ChainFkSolverPos_recursive(chain)


def joints_to_kdl(joint_values: List[float]):
    kdl_array = PyKDL.JntArray(NUM_OFF_JOINTS)

    for i in range(NUM_OFF_JOINTS):
        kdl_array[i] = joint_values[i]

    return kdl_array


def frame_to_list(frame: PyKDL.Frame) -> List[float]:
    rot = frame.M.GetQuaternion()

    return [frame.p.x(), frame.p.y(), frame.p.z(), rot[0], rot[1], rot[2], rot[3]]


def get_fk(solver: PyKDL.ChainFkSolverPos_recursive, joint_positions: List[float]) -> List[float]:
    end_frame = PyKDL.Frame()
    solver.JntToCart(joints_to_kdl(joint_positions), end_frame)

    return frame_to_list(end_frame)


def get_rgb_gradient(
    n: int,
    start_color: ColorRGBA = START_COLOR,
    end_color: ColorRGBA = END_COLOR,
) -> List[ColorRGBA]:
    colors = []
    for i in range(n):
        color = ColorRGBA()
        color.r = start_color.r + (end_color.r - start_color.r) * i / n
        color.g = start_color.g + (end_color.g - start_color.g) * i / n
        color.b = start_color.b + (end_color.b - start_color.b) * i / n
        color.a = start_color.a + (end_color.a - start_color.a) * i / n
        colors.append(color)

    return colors


class TrajectoryTracer:
    def __init__(self):
        self.name = rospy.get_name()

        self.marker_pub = rospy.Publisher('/trajectory_trace', MarkerArray, queue_size=1)

        rospy.Service(
            '/trace_trajectory',
            TraceTrajectory,
            self.trace_trajectory_callback,
        )
        self.rate = rospy.Rate(100)
        self.end_effector_link = None
        self.fk_solver = None

    def run(self):
        rospy.loginfo(f'{self.name}: ready to trace trajectories')
        while not rospy.is_shutdown():
            self.rate.sleep()

    def trace_trajectory_callback(self, req):
        self.clear_markers()

        if not rospy.has_param('robot_description'):
            rospy.logerr(f'{self.name}: robot_description not found')
            return False

        if self.end_effector_link != req.end_effector_link:
            self.end_effector_link = req.end_effector_link
            self.fk_solver = get_kdl_fk_solver(tip_link=self.end_effector_link)

        marker_array = MarkerArray()
        gradient = get_rgb_gradient(len(req.trajectory.joint_trajectory.points))

        for i, pose in enumerate(req.trajectory.joint_trajectory.points):
            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = rospy.Time.now()
            marker.color = gradient[i]
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = list_to_pose(get_fk(self.fk_solver, pose.positions))
            marker.scale.x = MARKER_SCALE
            marker.scale.y = MARKER_SCALE
            marker.scale.z = MARKER_SCALE
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

        return True

    def clear_markers(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)

        for i in range(10):
            self.marker_pub.publish(marker_array)
            rospy.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('trajectory_tracer', anonymous=True)
    tracer = TrajectoryTracer()
    tracer.run()