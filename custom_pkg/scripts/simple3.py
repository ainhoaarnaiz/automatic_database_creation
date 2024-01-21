import rospy
import plyfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np

def ply_to_pointcloud2(ply_path):
    # Read the .ply file
    ply_data = plyfile.PlyData.read(ply_path)

    # Extract vertices and colors
    vertices = np.vstack([ply_data['vertex']['x'], ply_data['vertex']['y'], ply_data['vertex']['z']]).T
    colors = np.vstack([ply_data['vertex']['red'], ply_data['vertex']['green'], ply_data['vertex']['blue']]).T

    # Create PointCloud2 message
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_link"  # Change this frame_id if needed

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('rgb', 12, PointField.UINT32, 1),
    ]

    points = np.zeros(vertices.shape[0], dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('rgb', np.uint32)
    ])

    points['x'] = vertices[:, 0]
    points['y'] = vertices[:, 1]
    points['z'] = vertices[:, 2]
    points['rgb'] = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]

    pc2_msg = PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        fields=fields,
        is_bigendian=False,
        point_step=16,  # Size of each point in bytes (4 for x, 4 for y, 4 for z, 4 for rgb)
        row_step=points.nbytes,
        data=points.tobytes()
    )

    return pc2_msg

def save_pointcloud_to_ply(pointcloud_msg, ply_filename):
    # Extract data from PointCloud2 message
    points = np.frombuffer(pointcloud_msg.data, dtype=np.float32).reshape((-1, 4))

    # Extract vertices and colors
    vertices = points[:, :3]
    colors = np.vstack([
        (points[:, 3] >> 16) & 0xff,
        (points[:, 3] >> 8) & 0xff,
        points[:, 3] & 0xff
    ]).T

    # Create PlyData
    ply_data = plyfile.PlyData([
        plyfile.PlyElement.describe(vertices, 'vertex', comments=['vertices']),
        plyfile.PlyElement.describe(colors, 'color', comments=['colors'])
    ])

    # Save to .ply file
    ply_data.write(ply_filename)

if __name__ == "__main__":
    rospy.init_node('ply_to_pointcloud2_publisher')

    # Replace 'your_file.ply' with the path to your .ply file
    ply_file_path = '/dev_ws/src/software_II_project/custom_pkg/captures/cropped_palito_01.ply'
    
    pointcloud_pub = rospy.Publisher('/pointcloud2', PointCloud2, queue_size=1)

    rate = rospy.Rate(1)  # Publish rate

    while not rospy.is_shutdown():
        pointcloud_msg = ply_to_pointcloud2(ply_file_path)
        pointcloud_pub.publish(pointcloud_msg)

        # Save PointCloud2 to .ply file
        save_pointcloud_to_ply(pointcloud_msg, '/home/ainhoaarnaiz/output_02.ply')

        rate.sleep()
