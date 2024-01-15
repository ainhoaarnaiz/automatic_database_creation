from copy import deepcopy
from typing import Dict

import cv2
import numpy as np
import open3d as o3d


def get_aruco_masks(image: np.ndarray) -> Dict[int, np.ndarray]:
    # Define the ArUco dictionary and detector parameters
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    # Detect the markers in the image
    corners, ids, rejected = detector.detectMarkers(image)

    masks = {}
    # If markers are detected
    if ids is not None:
        for i, corner in enumerate(corners):
            current_id = ids[i][0]
            contour = np.squeeze(corner).astype(int)  # Reshape corner list
            current_mask = np.zeros_like(image[:, :, 0], dtype=np.uint8)
            cv2.fillPoly(
                current_mask, [contour], 1
            )  # Fill the detected marker with white color on the mask
            masks[current_id] = current_mask

    return masks


def colorize_mask(mask: np.ndarray) -> np.ndarray:
    # Convert binary mask to a 3-channel visual mask
    visual_mask = np.stack([mask * 255] * 3, axis=-1)
    return visual_mask.astype(np.uint8)


def get_marker_pcd(
    pcd_o3d: o3d.geometry.PointCloud, masks: Dict[int, np.ndarray], width: int, height: int
) -> Dict[int, o3d.geometry.PointCloud]:
    marker_pcds = {}

    # Convert the o3d.geometry.PointCloud points to numpy ndarray with the given width and height
    pcd_points = np.asarray(pcd_o3d.points).reshape((height, width, 3))

    # Convert the o3d.geometry.PointCloud colors to numpy ndarray with the same shape
    pcd_colors = np.asarray(pcd_o3d.colors).reshape((height, width, 3))

    for m_id, mask in masks.items():
        # Find the bounding box of the mask
        rows, cols = np.where(mask)
        min_row, max_row = np.min(rows), np.max(rows)
        min_col, max_col = np.min(cols), np.max(cols)

        # Crop the point cloud data
        cropped_pcd_points = pcd_points[min_row : max_row + 1, min_col : max_col + 1]

        # Crop the color data
        cropped_pcd_colors = pcd_colors[min_row : max_row + 1, min_col : max_col + 1]

        # Crop the mask and expand its dimensions
        cropped_mask_expanded = mask[min_row : max_row + 1, min_col : max_col + 1, np.newaxis]

        # Apply the mask to the cropped point cloud and color data using broadcasting
        masked_pcd_points = cropped_pcd_points * cropped_mask_expanded
        masked_pcd_colors = cropped_pcd_colors * cropped_mask_expanded

        # Extract the valid points and colors
        valid_points = masked_pcd_points[cropped_mask_expanded[..., 0].astype(bool)]
        valid_colors = masked_pcd_colors[cropped_mask_expanded[..., 0].astype(bool)]

        # Create an Open3D point cloud, assign points and colors, and add to dictionary
        new_pcd_o3d = o3d.geometry.PointCloud()
        new_pcd_o3d.points = o3d.utility.Vector3dVector(valid_points)
        new_pcd_o3d.colors = o3d.utility.Vector3dVector(valid_colors)
        marker_pcds[m_id] = new_pcd_o3d

    return marker_pcds


def project_points_onto_plane(
    pcd: o3d.geometry.PointCloud, plane_model: list
) -> o3d.geometry.PointCloud:
    # Extract plane parameters
    a, b, c, d = plane_model
    plane_normal = np.array([a, b, c])
    plane_normal /= np.linalg.norm(plane_normal)  # Normalize the plane normal

    # For the point on the plane, we can arbitrarily pick z=0 to get the x and y.
    # If c is not zero: -d/c will give z coordinate
    # This is just an arbitrary point on the plane.
    x0, y0, z0 = 0, 0, -d / c if c != 0 else 0
    point_on_plane = np.array([x0, y0, z0])

    # Project each point in the point cloud onto the plane
    points = np.asarray(pcd.points)
    projected_points = (
        points - np.dot((points - point_on_plane), plane_normal)[:, np.newaxis] * plane_normal
    )

    # Create a new point cloud for the projected points
    projected_pcd = o3d.geometry.PointCloud()
    projected_pcd.points = o3d.utility.Vector3dVector(projected_points)
    return projected_pcd


def get_pcd_centroid_transform(pcd: o3d.geometry.PointCloud, plane_model: list) -> np.ndarray:
    centroid = np.mean(np.asarray(pcd.points), axis=0)

    z_axis = np.array(plane_model[:3])
    z_axis /= np.linalg.norm(z_axis)

    # Apply PCA to find principal direction
    centered_points = np.asarray(pcd.points) - centroid
    _, _, vh = np.linalg.svd(centered_points, full_matrices=True)
    x_axis = vh[0, :]

    # Ensure the X-axis is orthogonal to the Z-axis
    x_axis -= x_axis.dot(z_axis) * z_axis
    x_axis /= np.linalg.norm(x_axis)

    # Compute the Y-axis as the cross product of Z and X
    y_axis = np.cross(z_axis, x_axis)

    # Create the rotation matrix
    rot_matrix = np.array([x_axis, y_axis, z_axis])

    # Create the transformation matrix
    trans_vector = np.expand_dims(centroid, axis=-1)
    return np.block([[rot_matrix, trans_vector], [0, 0, 0, 1]])


def get_square_alignment_transform(pcd: o3d.geometry.PointCloud, plane_model: list) -> np.ndarray:
    # 1. Compute the centroid of the point cloud
    centroid = np.mean(np.asarray(pcd.points), axis=0)

    # 2. Set the Z-axis as the plane normal
    z_axis = np.array(plane_model[:3])
    z_axis /= np.linalg.norm(z_axis)

    # 3. Use PCA to get the principal direction
    _, _, vh = np.linalg.svd(np.asarray(pcd.points) - centroid, full_matrices=True)

    # Make this principal direction orthogonal to z_axis
    x_axis = vh[0, :] - np.dot(vh[0, :], z_axis) * z_axis
    x_axis /= np.linalg.norm(x_axis)

    # Compute y_axis as the cross product of z_axis and x_axis
    y_axis = np.cross(z_axis, x_axis)

    # Create the rotation matrix using x_axis, y_axis, and z_axis
    rot_matrix = np.array([x_axis, y_axis, z_axis])

    # 4. Create the transformation matrix
    transform = np.eye(4)
    transform[:3, :3] = rot_matrix.T
    transform[:3, 3] = centroid

    return transform


# cv2.namedWindow("find_marker", cv2.WINDOW_NORMAL)

rgb = cv2.imread("/home/v/capture/marker_rgb.png")
# cv2.imshow("find_marker", rgb)
# cv2.waitKey(0)

pcd = o3d.io.read_point_cloud("/home/v/capture/marker_pcd.ply")
o3d.visualization.draw_geometries([deepcopy(pcd).remove_non_finite_points()])

masks = get_aruco_masks(rgb)

# for _, mask in masks.items():  # Extract just the mask from the dictionary item
#     cv2.imshow("find_marker", np.vstack([rgb, colorize_mask(mask)]))
#     cv2.waitKey(0)
# cv2.destroyAllWindows()

height = rgb.shape[0]
width = rgb.shape[1]
marker_pcds = get_marker_pcd(pcd, masks, width, height)
marker_pcds = {m_id: pcd_o3d.remove_non_finite_points() for m_id, pcd_o3d in marker_pcds.items()}

o3d.visualization.draw_geometries(list(marker_pcds.values()))

for _, marker in marker_pcds.items():
    plane_model, inliers = marker.segment_plane(
        distance_threshold=0.001, ransac_n=3, num_iterations=1000
    )
    # marker = project_points_onto_plane(marker, plane_model)

    # marker_tf = get_square_alignment_transform(marker, plane_model)
    marker_tf = get_pcd_centroid_transform(marker, plane_model)
    tf_vis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1).transform(marker_tf)

    o3d.visualization.draw_geometries(
        [
            marker.select_by_index(inliers).paint_uniform_color([1.0, 0, 0]),
            marker.select_by_index(inliers, invert=True),
            tf_vis,
        ]
    )
