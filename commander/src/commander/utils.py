from typing import List

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_commander import PlanningSceneInterface
from moveit_commander.conversions import list_to_pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, Header
from yaml import safe_load

from commander.srv import VisualizePoses


def display_poses(frame_id: str, poses: List[Pose]) -> None:
    rospy.wait_for_service('/visualize_poses', timeout=10)
    visualize_poses = rospy.ServiceProxy('/visualize_poses', VisualizePoses)
    visualize_poses(frame_id, poses)


def poses_from_yaml(yaml_file: str) -> List[Pose]:
    poses = []

    with open(yaml_file, 'r', encoding='utf-8') as file:
        y = safe_load(file)
        for pose in y.get('path'):
            p = [
                pose['position'][0],
                pose['position'][1],
                pose['position'][2],
                pose['quaternion'][0],
                pose['quaternion'][1],
                pose['quaternion'][2],
                pose['quaternion'][3],
            ]
            poses.append(list_to_pose(p))

    return poses


def load_scene(frame_id: str = 'base_link') -> None:
    scene_interface = PlanningSceneInterface()
    scene = PlanningScene()
    scene.is_diff = True

    # ground cube
    co = CollisionObject()
    co.header.frame_id = frame_id
    co.header.stamp = rospy.Time.now()
    co.pose = Pose(
        position=Point(0.0, 0.0, -0.691 / 2.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )
    co.id = 'ground_cube'
    co.operation = CollisionObject.ADD
    co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.635, 0.760, 0.690]))
    scene.world.collision_objects.append(co)

    oc = ObjectColor()
    oc.id = 'ground_cube'
    oc.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.7)
    scene.object_colors.append(oc)

    scene_interface.apply_planning_scene(scene)
