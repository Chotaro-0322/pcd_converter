import numpy as np
import struct
import glob
from open3d import *
from mayavi.mlab import *
import mayavi.mlab as mlab
from tqdm import tqdm

def convert_kitti_bin_to_pcd(binFilePath):
    size_float = 4
    list_pcd = []
    with open(binFilePath, "rb") as f:
        byte = f.read(size_float * 4)
        while byte:
            x, y, z, intensity = struct.unpack("ffff", byte)
            list_pcd.append([x, y, z])
            byte = f.read(size_float * 4)
    np_pcd = np.asarray(list_pcd)
    pcd = geometry.PointCloud()
    pcd.points = utility.Vector3dVector(np_pcd)
    return pcd

def convert_kitti_pcd_to_bin(pcd_data, bin_name):
    with open(bin_name, "wb") as f:
        for i in range(len(pcd_data)):
            # print("bin input is", pcd_data[i, 0])
            f.write(pcd_data[i, 0].astype(np.float32))
            f.write(pcd_data[i, 1].astype(np.float32))
            f.write(pcd_data[i, 2].astype(np.float32))
            f.write(np.zeros(1).astype(np.float32))

bin_list = glob.glob("./kitti/testing/velodyne/000000.bin")
print("path list :", bin_list) 

for bin_path in tqdm(bin_list):
    # print("bin_path", bin_path)
    pcd = convert_kitti_bin_to_pcd(bin_path)

    np_pcd = np.asarray(pcd.points) # convert pcd.points to numpy
    # print("np_pcd shape", np_pcd.shape)
    delete_num = np_pcd.shape[0] % 32 # padding
    # print("delete_num is ", delete_num)
    if delete_num != 0:
        np_pcd = np_pcd[:-delete_num]
        # print("np_pcd padding", np_pcd.shape)
    pcd_h = np_pcd.reshape((32, -1, 3)) # reshape using lidar layer

    # print("pcd data : ", pcd_h.shape)

    pcd_h = np.asarray([pcd_tmp for i, pcd_tmp in enumerate(pcd_h) if i % 2 == 0])
    
    # print("pcd data : ", pcd_h.shape)
    pcd_down_sample = pcd_h.reshape((-1, 3))
    # print("pcd_down_sample : ", pcd_down_sample.shape)

    # print("bin path is ", "./vlp16_kitti/" + bin_path[-10:])
    convert_kitti_pcd_to_bin(pcd_down_sample, "./vlp16_kitti_test2/" + bin_path[-10:])

    points3d(pcd_down_sample[:, 0], pcd_down_sample[:, 1], pcd_down_sample[:, 2], colormap="copper", scale_factor=0.05, line_width=0.1)
    mlab.show(stop=True)








# TEST!!!!
# bin_list = glob.glob("./vlp16_kitti/000000.bin")
# print("path list :", bin_list) 
# for bin_path in bin_list:
#     pcd = convert_kitti_bin_to_pcd(bin_path)

#     np_pcd = np.asarray(pcd.points) # convert pcd.points to numpy
#     # delete_num = np_pcd.shape[0] % 32 # padding
#     # np_pcd = np_pcd[:-delete_num]
#     # pcd_h = np_pcd.reshape((32, -1, 3)) # reshape using lidar layer

#     print("pcd data : ", np_pcd.shape)
#     # pcd_down_sample = pcd_h.reshape((-1, 3))

#     points3d(np_pcd[:, 0], np_pcd[:, 1], np_pcd[:, 2], colormap="copper", scale_factor=0.05, line_width=0.1)
#     mlab.show(stop=True)
