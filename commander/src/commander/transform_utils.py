from copy import deepcopy
from typing import List

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, Vector3
from pytransform3d import rotations as tf3d_r
from pytransform3d import transformations as tf3d_t

# Pose to matrix and back
# -----------------------


def pose_to_transform(pose: Pose) -> np.ndarray:
    pq = [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
    ]
    pq[3:] = tf3d_r.check_quaternion(pq[3:])
    pq = tf3d_t.check_pq(pq)

    return tf3d_t.transform_from_pq(pq)


def transform_to_pose(transform: np.ndarray) -> Pose:
    transform = tf3d_t.check_transform(transform)
    pq = tf3d_t.pq_from_transform(transform)
    pose = Pose()
    pose.position = Point(pq[0], pq[1], pq[2])
    pose.orientation = Quaternion(pq[4], pq[5], pq[6], pq[3])

    return pose


def assert_quaternions_almost_equal(q1, q2, tolerance=1e-5):
    dot_product = np.abs(np.dot(q1, q2))
    assert dot_product > 1 - tolerance, f"Quaternions are not almost equal: {dot_product}"


def create_translation_matrix(translation: List[float]) -> np.ndarray:
    assert len(translation) == 3

    translation_matrix = np.eye(4)
    translation_matrix[:3, 3] = translation

    return translation_matrix


def create_rotation_matrix(rotations: List[float]) -> np.ndarray:
    assert len(rotations) == 3

    transformation_matrix = np.eye(4)
    for basis, angle in enumerate(rotations):
        rotation_matrix = np.eye(4)
        rotation_matrix[:3, :3] = tf3d_r.active_matrix_from_angle(basis, angle)
        transformation_matrix = np.matmul(transformation_matrix, rotation_matrix)

    return transformation_matrix


def apply_transformation(pose: Pose, transformation: np.ndarray) -> Pose:
    assert transformation.shape == (4, 4)

    tf = pose_to_transform(pose)
    tf = np.matmul(tf, transformation)
    return transform_to_pose(tf)


def translate_pose(pose: Pose, translation: List[float]) -> Pose:
    translation_matrix = create_translation_matrix(translation)
    return apply_transformation(pose, translation_matrix)


def orient_poses(target_pose: Pose, poses: List[Pose]) -> List[Pose]:
    poses = deepcopy(poses)
    tf = pose_to_transform(target_pose)
    return [transform_to_pose(np.matmul(tf, pose_to_transform(pose))) for pose in poses]


def transform_to_tf(transform: np.ndarray, frame_id: str, name: str) -> Transform:
    tf = Transform()
    tf.transform.translation = Vector3(*transform[:3, 3])
    tf.transform.rotation = Quaternion(*tf3d_r.quaternion_from_(transform[:3, 3]))

    return tf
