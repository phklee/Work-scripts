# -*- coding: UTF-8 -*-
import sys
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import pprint
import copy
import itertools
from tkinter import _flatten
import open3d as o3d
from numpy import mat

from scipy.spatial.transform import Rotation as R

np.set_printoptions(suppress=True)

img = f'./test_data_1/2_H90l_1656648495300.jpg'

extr_R_ = np.array([[   0.704625, -0.00152066,    0.709579],
 [ 0.00368944,    0.999992, -0.00152066],
 [  -0.709571,  0.00368944,    0.704625]]
)


heading = {
    "w": 0.9228708204710818,
    "z": 0.001407881828232372,
    "y": 0.3851045039042997,
    "x": 0.001407881828232372
}

poistion = {
    "z": -1.48,
    "x": 0.22,
    "y": -0.06
}


#四元数组转旋转矩阵
Rq = np.asarray([heading['x'], heading['y'], heading['z'], heading['w']])
r = R.from_quat(Rq)
extr_R_ = r.as_matrix()

extr_R_init_ = np.array([[1.19209e-07, -1, 0],
                         [0, 1.19209e-07, -1],
                         [1, 0, 1.19209e-07]])

extr_t_ = np.array([[ 0.22],
 [-0.06],
 [-1.48]])

k = np.array([825.872911, 0, 645.475697, 0, 827.100951, 350.396819, 0, 0, 1]).reshape(3,3)

def read_pcd(file_path):
    pcd = o3d.io.read_point_cloud(file_path)
    points = np.asarray(pcd.points)

    return points

scan = read_pcd("./test_data_1/1656648495300.pcd")
# read raw data from binary
# scan = np.fromfile(binary, dtype=np.float32).reshape((-1, 4))
points = scan[:, 0:3]  # lidar xyz (front, left, up)

#velo = np.insert(points, 3, 1, axis=1).T
velo = points.T

cam = np.dot(extr_R_,np.dot(extr_R_init_,velo)+extr_t_)

cam = np.dot(k,cam)
# cam = np.delete(cam, np.where(cam[2, :] < 0)[1], axis=1)
# get u,v,z
cam[:2] /= cam[2, :]

plt.figure(figsize=(12, 5), dpi=96, tight_layout=True)
png = mpimg.imread(img)
IMG_H, IMG_W, _ = png.shape
# restrict canvas in range
plt.axis([0, IMG_W, IMG_H, 0])
plt.imshow(png)
# filter point out of canvas
u, v, z = cam
u_out = np.logical_or(u < 0, u > IMG_W)
v_out = np.logical_or(v < 0, v > IMG_H)
outlier = np.logical_or(u_out, v_out)
cam = np.delete(cam, np.where(outlier), axis=1)
# generate color map from depth
name = "2_H60_670"
u, v, z = cam
plt.scatter([u], [v], c=[z], cmap='rainbow_r', alpha=0.5, s=2)
plt.title(name)
plt.savefig('./{name}.png', bbox_inches='tight')
plt.show()
