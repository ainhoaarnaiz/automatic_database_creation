#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import open3d as o3d
import numpy as np

def read_ply_file(file_path):
    cloud = o3d.io.read_point_cloud(file_path)
    
    # Convert color values to the range [0, 255]
    colors = np.asarray(cloud.colors) * 255
    cloud.colors = o3d.utility.Vector3dVector(colors)

    return cloud


def save_pointcloud_as_ply(pointcloud_msg):
    # Parse point cloud data
    points = []
    colors = []

    for point in point_cloud2.read_points(pointcloud_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
        x, y, z, rgb = point

        # Extract individual color channels
        r = (rgb >> 16) & 0xFF
        g = (rgb >> 8) & 0xFF
        b = rgb & 0xFF

        # Normalize colors to the range [0, 1]
        r_normalized = r / 255.0
        g_normalized = g / 255.0
        b_normalized = b / 255.0

        points.append([x, y, z])
        colors.append([r_normalized, g_normalized, b_normalized])

    # Convert lists to NumPy arrays
    points_array = np.asarray(points, dtype=np.float32)
    colors_array = np.asarray(colors, dtype=np.float32)

    # Create an Open3D PointCloud
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points_array)
    point_cloud.colors = o3d.utility.Vector3dVector(colors_array)

    # Save the PointCloud as PLY
    o3d.io.write_point_cloud("/home/ainhoaarnaiz/output_02.ply", point_cloud)


def open3d_to_pointcloud2(o3d_cloud):
    # Convert Open3D PointCloud to sensor_msgs/PointCloud2
    points = np.asarray(o3d_cloud.points)
    colors = (np.asarray(o3d_cloud.colors) * 255).astype(np.uint8)

    # Create PointCloud2 message
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_link"  # Change this to your desired frame_id

    pcl_msg = PointCloud2()
    pcl_msg.header = header
    pcl_msg.height = 1
    pcl_msg.width = points.shape[0]
    pcl_msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),  # RGB values as a single uint32
    ]
    pcl_msg.is_bigendian = False
    pcl_msg.point_step = 16  # 4 (x) + 4 (y) + 4 (z) + 4 (rgb)
    pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width
    pcl_msg.is_dense = True

    # Combine RGB values into a single UINT32 field
    rgb_packed = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
    rgb_packed = rgb_packed.astype(np.uint32)

    # Flatten the arrays and assign them to the PointCloud2 message
    pcl_msg.data = np.column_stack((points, rgb_packed)).astype(np.float32).tobytes()

    save_pointcloud_as_ply(pcl_msg)

    return pcl_msg




def publish_pointcloud(o3d_cloud, publisher):
    cloud_msg = open3d_to_pointcloud2(o3d_cloud)
    publisher.publish(cloud_msg)

def main():
    rospy.init_node('ply_to_pointcloud_publisher', anonymous=True)
    
    # Change the file path to the location of your .ply file
    ply_file_path = '/dev_ws/src/software_II_project/custom_pkg/captures/cropped_palito_01.ply'

    o3d_cloud = read_ply_file(ply_file_path)

    pub = rospy.Publisher('/pointcloud2', PointCloud2, queue_size=10)

    rate = rospy.Rate(0.1)  # Adjust the rate based on your requirements

    while not rospy.is_shutdown():
        publish_pointcloud(o3d_cloud, pub)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
