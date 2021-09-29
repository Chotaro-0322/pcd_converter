import open3d as o3d
import numpy as np
import glob
from mayavi.mlab import *
import mayavi.mlab as mlab

def affine_rotation(pcd_points, rotate, direction):
    if direction == "x":
        pcd_x = pcd_points[:, 0]
        pcd_y = pcd_points[:, 1] * np.cos(np.deg2rad(rotate)) - pcd_points[:, 2] * np.sin(np.deg2rad(rotate))
        pcd_z = pcd_points[:, 1] * np.sin(np.deg2rad(rotate)) + pcd_points[:, 2] * np.cos(np.deg2rad(rotate))
        return np.stack([pcd_x, pcd_y, pcd_z], 1)
    elif direction == "y":
        pcd_x = pcd_points[:, 0] * np.cos(np.deg2rad(rotate)) + pcd_points[:, 2] * np.sin(np.deg2rad(rotate))
        pcd_y = pcd_points[:, 1]
        pcd_z = -1 * pcd_points[:, 0] * np.sin(np.deg2rad(rotate)) + pcd_points[:, 2] * np.cos(np.deg2rad(rotate))
        return np.stack([pcd_x, pcd_y, pcd_z], 1)
    elif direction == "z":
        pcd_x = pcd_points[:, 0] * np.cos(np.deg2rad(rotate)) - pcd_points[:, 1] * np.sin(np.deg2rad(rotate))
        pcd_y = pcd_points[:, 0] * np.sin(np.deg2rad(rotate)) + pcd_points[:, 1] * np.cos(np.deg2rad(rotate))
        pcd_z = pcd_points[:, 2]  
        return np.stack([pcd_x, pcd_y, pcd_z], 1)

def affine_move(pcd_points, x = 0, y = 0, z = 0):
    pcd_x = pcd_points[:, 0] + x
    pcd_y = pcd_points[:, 1] + y
    pcd_z = pcd_points[:, 2] + z
    return np.stack([pcd_x, pcd_y, pcd_z], 1)
    

pcd_path = glob.glob("*.pcd")

for pcd_name in pcd_path:
    pcd_load = o3d.io.read_point_cloud(pcd_name)

    xyz_load = np.asarray(pcd_load.points)
    #print("xyz_load : ", xyz_load.shape)

    xyz_load = affine_rotation(xyz_load, 30, "z")
    xyz_load = affine_move(xyz_load, x = 10, y = 10, z = 10)
    
    points3d(xyz_load[:, 0], xyz_load[:, 1], xyz_load[:, 2], colormap="copper", scale_factor=0.05, line_width=0.1)
    mlab.show(stop=True)
    # np.save("{}.npy".format(pcd_name[:-4]), xyz_load)
    # print("{}.npy is saved".format(pcd_name[:-4]))



