#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud, ChannelFloat32
import geometry_msgs.msg
import open3d as o3d
import numpy as np

def save_pointcloud_as_ply(pointcloud_msg):
    # Parse point cloud data
    points = []

    for point in pointcloud_msg.points:
        x, y, z = point.x, point.y, point.z
        points.append([x, y, z])

    # Convert list to NumPy array
    points_array = np.asarray(points, dtype=np.float32)

    # Create an Open3D PointCloud
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points_array)

    # Save the PointCloud as PLY
    o3d.io.write_point_cloud("/home/ainhoaarnaiz/output_01.ply", point_cloud)

def read_ply_file(file_path):
    cloud = o3d.io.read_point_cloud(file_path)
    return cloud

def open3d_to_pointcloud(o3d_cloud):
    # Convert Open3D PointCloud to sensor_msgs/PointCloud
    points = np.asarray(o3d_cloud.points)
    colors = np.asarray(o3d_cloud.colors) * 255.0  # Assuming colors are in the range [0, 255]

    cloud_msg = PointCloud()
    cloud_msg.header.stamp = rospy.Time.now()
    cloud_msg.header.frame_id = "base_link"  # Change this to your desired frame_id

    for i in range(points.shape[0]):
        point = [points[i, 0], points[i, 1], points[i, 2]]

        # Create a Point32 message for each point
        point_msg = geometry_msgs.msg.Point32()
        point_msg.x, point_msg.y, point_msg.z = point

        # Add point coordinates to the message
        cloud_msg.points.append(point_msg)

        # Add color data to the message
        color_channel = ChannelFloat32()
        # Assuming colors are in the range [0, 255]
        color_channel.values.append(colors[i, 0])
        color_channel.values.append(colors[i, 1])
        color_channel.values.append(colors[i, 2])
        cloud_msg.channels.append(color_channel)

    save_pointcloud_as_ply(cloud_msg)

    return cloud_msg

def publish_pointcloud(o3d_cloud, publisher):
    cloud_msg = open3d_to_pointcloud(o3d_cloud)
    publisher.publish(cloud_msg)

def main():
    rospy.init_node('ply_to_pointcloud_publisher', anonymous=True)
    
    # Change the file path to the location of your .ply file
    ply_file_path = '/dev_ws/src/software_II_project/custom_pkg/captures/cropped_01.ply'

    o3d_cloud = read_ply_file(ply_file_path)

    pub = rospy.Publisher('/pointcloud', PointCloud, queue_size=10)

    rate = rospy.Rate(0.1)  # Adjust the rate based on your requirements

    while not rospy.is_shutdown():
        publish_pointcloud(o3d_cloud, pub)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
