#!/usr/bin/env python3

import rospy
import open3d as o3d
import numpy as np
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud, ChannelFloat32

def crop_point_cloud(file_path, save_path):
    while not rospy.is_shutdown():
        try:
            # Load your point cloud
            point_cloud = o3d.io.read_point_cloud(file_path)

            # Define the coordinates of the bounding box corners
            point1 = [0.72, -0.5, 0.0038]
            point2 = [0.87, -0.5, 0.0038]
            point3 = [0.72, 0.5, 0.0038]
            point4 = [0.87, 0.5, 0.0038]

            # Assuming a small thickness for the bounding box along Z-axis
            z_thickness = 0.1  # Adjust this value as needed

            # Find min and max bounds for the AABB
            min_bound = [min(point1[0], point3[0]), min(point1[1], point2[1]), min(point1[2], point2[2])]
            max_bound = [max(point2[0], point4[0]), max(point3[1], point4[1]), max(point3[2], point4[2]) + z_thickness / 2]

            # Create the axis-aligned bounding box
            aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)

            # Crop the point cloud
            cropped_point_cloud = point_cloud.crop(aabb)

            # o3d.visualization.draw_geometries([cropped_point_cloud])
            # Save the cropped point cloud as a .ply file
            o3d.io.write_point_cloud(save_path, cropped_point_cloud)

            # Convert Open3D PointCloud to sensor_msgs/PointCloud
            points = np.asarray(cropped_point_cloud.points)
            colors = np.asarray(cropped_point_cloud.colors) * 255.0  # Assuming colors are in the range [0, 255]

            cloud_msg = PointCloud()
            cloud_msg.header.stamp = rospy.Time.now()
            cloud_msg.header.frame_id = "base_link"  # Change this to your desired frame_id

            for i in range(points.shape[0]):
                point = [points[i, 0], points[i, 1], points[i, 2]]

                # Create a Point32 message for each point
                point_msg = Point32()
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

            # Publish the cropped point cloud
            pub = rospy.Publisher('/cropped_point_cloud', PointCloud, queue_size=10)
            pub.publish(cloud_msg)

        except FileNotFoundError:
            rospy.loginfo(f"File not found: {file_path}. Waiting for the file to be available.")
            rospy.sleep(5)  # Adjust the sleep interval as needed

if __name__ == '__main__':
    rospy.init_node('point_cloud_cropper_node')
    ply_file_path = rospy.get_param("~ply_file_path", "/dev_ws/src/software_II_project/custom_pkg/captures/raw.ply")
    ply_save_path = rospy.get_param("~ply_save_path", "/dev_ws/src/software_II_project/custom_pkg/captures/cropped.ply")
    try:
        crop_point_cloud(ply_file_path, ply_save_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
