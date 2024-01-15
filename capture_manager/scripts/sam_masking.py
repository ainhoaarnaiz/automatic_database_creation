from copy import deepcopy
from pathlib import Path
from typing import Dict

import cv2
import numpy as np
import open3d as o3d


def load_masks(path: Path) -> Dict[int, np.ndarray]:
    masks = {}
    for mask_path in path.glob("*.png"):
        mask_id = int(mask_path.stem)
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        masks[mask_id] = mask
    return masks


def mask_pcd(pcd: o3d.geometry.PointCloud, mask: np.ndarray) -> o3d.geometry.PointCloud:
    # Assert to ensure the dimensions match
    assert mask.shape[0] * mask.shape[1] == len(
        pcd.points
    ), "Mask dimensions don't match with point cloud size."

    pcd_np = np.asarray(pcd.points)
    colors_np = np.asarray(pcd.colors)

    # Create a boolean mask with False where mask == 0 and True elsewhere
    bool_mask = mask.reshape(-1) != 0

    # Mask the point cloud and colors using the boolean mask
    masked_pcd_np = pcd_np[bool_mask]
    masked_colors_np = colors_np[bool_mask]

    # Convert the masked point cloud back to Open3D format
    masked_pcd = o3d.geometry.PointCloud()
    masked_pcd.points = o3d.utility.Vector3dVector(masked_pcd_np)
    masked_pcd.colors = o3d.utility.Vector3dVector(masked_colors_np)

    return masked_pcd


def color_masked_points_red(
    pcd: o3d.geometry.PointCloud, mask: np.ndarray
) -> o3d.geometry.PointCloud:
    # Assert to ensure the dimensions match
    assert mask.shape[0] * mask.shape[1] == len(
        pcd.points
    ), "Mask dimensions don't match with point cloud size."

    # Make a deepcopy so as not to modify the original point cloud
    colored_pcd = o3d.geometry.PointCloud(pcd)

    # Convert points and colors to numpy arrays for efficient manipulation
    points_np = np.asarray(colored_pcd.points)
    colors_np = np.asarray(colored_pcd.colors)

    # Find indices where the mask is 0
    masked_indices = np.where(mask.reshape(-1) == 0)

    # Set the color of these points to red
    colors_np[masked_indices] = [1, 0, 0]  # RGB for red

    # Update the colors of the point cloud
    colored_pcd.colors = o3d.utility.Vector3dVector(colors_np)

    return colored_pcd


pcd = o3d.io.read_point_cloud("/home/v/capture/sam_pcd.ply")
rgb = cv2.imread("/home/v/capture/sam_rgb.png")
sam_masks = {"0": cv2.imread("/home/v/capture/mask19.png", cv2.IMREAD_GRAYSCALE)}

# print(rgb.shape)
# print(np.asarray(pcd.points).shape)
assert rgb.shape[1] * rgb.shape[0] == len(pcd.points)


cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
# sam_masks = load_masks(Path("/home/v/capture/masks"))




pcd_np = np.asarray(pcd.points)
colors_np = np.asarray(pcd.colors)

for mask_id, mask in sam_masks.items():
    masked_rgb = rgb.copy()
    masked_rgb[mask != 0] = [0, 0, 0]  # Set the masked regions to black for all channels at once
    cv2.imshow("mask", masked_rgb)
    cv2.waitKey(0)

    masked_pcd = mask_pcd(pcd, mask)
    o3d.visualization.draw_geometries([masked_pcd.remove_non_finite_points()])
    colored_pcd = color_masked_points_red(pcd, mask)
    o3d.visualization.draw_geometries([colored_pcd.remove_non_finite_points()])


cv2.destroyAllWindows()
