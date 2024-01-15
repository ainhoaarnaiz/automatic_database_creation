import math
from typing import List

import rospy
from geometry_msgs.msg import Vector3
from ur_msgs.srv import SetIO, SetIORequest, SetPayload, SetPayloadRequest


class UrRobot:
    def __init__(self) -> None:
        self.set_IO_srv = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        self.set_payload_srv = rospy.ServiceProxy('/ur_hardware_interface/set_payload', SetPayload)

    def set_do(self, do: int, state: int) -> bool:
        return self.set_IO_srv(SetIORequest(fun=1, pin=do, state=state))

    def set_payload(self, mass: float, cog: Vector3) -> bool:
        return self.set_payload_srv(SetPayloadRequest(mass=mass, center_of_gravity=cog))


def _norm2(a, b, c=0.0):
    return math.sqrt(a**2 + b**2 + c**2)


def ur_axis_angle_to_quat(axis_angle: List[float]) -> List[float]:
    # https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Unit_quaternions
    angle = _norm2(*axis_angle)
    axis_normed = [axis_angle[0] / angle, axis_angle[1] / angle, axis_angle[2] / angle]
    s = math.sin(angle / 2)
    return [
        s * axis_normed[0],
        s * axis_normed[1],
        s * axis_normed[2],
        math.cos(angle / 2),
    ]  # xyzw


def quat_to_ur_axis_angle(quaternion: List[float]) -> List[float]:
    # https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Unit_quaternions
    # quaternion must be [xyzw]
    angle = 2 * math.atan2(
        _norm2(quaternion[0], quaternion[1], quaternion[2]),
        quaternion[3],
    )
    if abs(angle) > 1e-6:
        axis_normed = [
            quaternion[0] / math.sin(angle / 2),
            quaternion[1] / math.sin(angle / 2),
            quaternion[2] / math.sin(angle / 2),
        ]
    else:
        axis_normed = 0.0
    return [axis_normed[0] * angle, axis_normed[1] * angle, axis_normed[2] * angle]
