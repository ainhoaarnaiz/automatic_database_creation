from math import pi

from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import euler_from_quaternion

from commander.transform_utils import apply_transformation, create_rotation_matrix

# quat = [0.525482745502, -0.525482745496, 0.473146789252, -0.473146789259]

# print(euler_from_quaternion(quat, 'sxyz'))


target0 = Pose(
    position=Point(0.5, -0.4, 0.4),
    orientation=Quaternion(1.0, 0.0, 0.0, 0.0),
)

rot = create_rotation_matrix([0, 0, pi / 2])
target0 = apply_transformation(target0, rot)

print(target0)