import struct
from pathlib import Path

import cv2
import message_filters
import numpy as np
import open3d as o3d
import rospy
from cv_bridge import CvBridge, CvBridgeError
from capture_manager.srv import CaptureToFile
from sensor_msgs.msg import Image, PointCloud2

from commander.transform_utils import pose_to_transform


class KaCaptureManager:
    OUTPUT_DIR = Path("/home/v/capture")

    def __init__(self) -> None:
        self.name = "capture_manager"
        rospy.init_node(self.name)
        self.rgb_topic = rospy.get_param("/capture_manager/rgb_topic")
        self.pcd_topic = rospy.get_param("/capture_manager/pcd_topic")

        rospy.loginfo(f"{self.name} > rgb_topic: {self.rgb_topic}, pcd_topic: {self.pcd_topic}")

        self.rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        self.pcd_sub = message_filters.Subscriber(self.pcd_topic, PointCloud2)

        ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.pcd_sub], queue_size=10, slop=0.1
        )
        ts.registerCallback(self.callback)

        self.latest_rgb = None
        self.latest_pcd = None

        self.capture_to_file = rospy.Service(
            "capture_to_file", CaptureToFile, self.capture_to_file_cb
        )
        self.cv_bridge = CvBridge()

    def run(self):
        rospy.loginfo(f"{self.name} running")
        while not rospy.is_shutdown():
            rospy.spin()

    def callback(self, rgb, pcd):
        self.latest_rgb = rgb
        self.latest_pcd = pcd


    def capture_to_file_cb(self, req):
        if self.latest_rgb and self.latest_pcd:
            try:
                rgb_image = self.cv_bridge.imgmsg_to_cv2(self.latest_rgb, desired_encoding="bgr8")
            except CvBridgeError as e:
                rospy.logerr(e)

            pcd = self.pointcloud2_to_o3d(self.latest_pcd)
            np.savetxt(f"{req.path}_tf.txt", pose_to_transform(req.capture_pose))

            return all(
                (
                    cv2.imwrite(f"{req.path}_rgb.png", rgb_image),
                    o3d.io.write_point_cloud(f"{req.path}_pcd.ply", pcd),
                )
            )
        rospy.logwarn(f"{self.name} > No synchronized rgb pcd pair received yet!")
        return False

    @staticmethod
    def pointcloud2_to_o3d(pointcloud2_msg):
        # Create an open3d point cloud object
        pc_o3d = o3d.geometry.PointCloud()

        # Extract byte data from the PointCloud2 message
        byte_data = pointcloud2_msg.data

        # Decode the byte data to extract points and colors
        points = []
        colors = []
        for i in range(0, len(byte_data), 32):  # Assuming point step is 32 bytes
            # Extract x, y, z values
            x = struct.unpack("f", byte_data[i : i + 4])[0]
            y = struct.unpack("f", byte_data[i + 4 : i + 8])[0]
            z = struct.unpack("f", byte_data[i + 8 : i + 12])[0]

            # Extract rgb values (packed into a single float32)
            packed_rgb = struct.unpack("I", byte_data[i + 16 : i + 20])[0]

            # Extract individual RGB channels
            r = ((packed_rgb >> 16) & 0xFF) / 255.0
            g = ((packed_rgb >> 8) & 0xFF) / 255.0
            b = (packed_rgb & 0xFF) / 255.0

            points.append([x, y, z])
            colors.append([r, g, b])

        # Assign the points and colors to the open3d point cloud object
        pc_o3d.points = o3d.utility.Vector3dVector(np.array(points))
        pc_o3d.colors = o3d.utility.Vector3dVector(np.array(colors))

        return pc_o3d


if __name__ == "__main__":
    ka_capture_manager = KaCaptureManager()
    ka_capture_manager.run()
