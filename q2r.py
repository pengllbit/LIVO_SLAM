from multiprocessing import Pool
import numpy as np
import sys
import os
import json
import queue
from os.path import exists
from numpy import arctan2, arcsin
import matplotlib.pyplot as plt

# 四元数转旋转矩阵
# q_x = -0.000368795110496128 
# q_y = -9.40514960083596e-05 
# q_z = -0.00871051512264581 
# q_w = 0.999962001317006
# #roll, pitch, yaw 弧度
# roll = arctan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x ** 2 + q_y ** 2))
# pitch = arcsin(2 * (q_w * q_y - q_z * q_x))
# yaw = arctan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y ** 2 + q_z ** 2))
# 计算旋转矩阵 
# Rx = np.array([[1, 0, 0],
#             [0, np.cos(roll), -np.sin(roll)],
#             [0, np.sin(roll), np.cos(roll)]])
# Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
#             [0, 1, 0],
#             [-np.sin(pitch), 0, np.cos(pitch)]])
# Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
#             [np.sin(yaw), np.cos(yaw), 0],
#             [0, 0, 1]])
# R_lidar_2_world = Rz.dot(Ry).dot(Rx)
# print(R_lidar_2_world)

# R_lisoam = np.array([[ 9.99848236e-01 , 1.74204376e-02 ,-1.81671053e-04 , -0.080241],
#             [-1.74202989e-02 , 9.99847982e-01 , 7.39200668e-04 , -0.0563],
#             [ 1.94520635e-04 ,-7.35923720e-04 , 9.99999710e-01 , -0.0094],
#             [0 , 0 , 0 , 1]])

# R_icp = np.array([[1.00 , 0.017 , -0.0000001 , -0.025],
#             [ -0.017 , 1.000 , -0.00001 , -0.045],
#             [ 0.000001 ,0.000001, 1.0 , 0.001],
#             [0 , 0 , 0 ,1 ]])

# R = np.dot(np.linalg.inv(R_lisoam) , R_icp)
# print(R)
def euler_from_matrix(vector_transformed):
    R = vector_transformed

    # 计算欧拉角
    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
    yaw = np.arctan2(R[1, 0], R[0, 0])

    # 将弧度转换为角度
    roll_deg = np.degrees(roll)
    pitch_deg = np.degrees(pitch)
    yaw_deg = np.degrees(yaw)

    print("Roll (X-axis): {:.2f} degrees".format(roll_deg))
    print("Pitch (Y-axis): {:.2f} degrees".format(pitch_deg))
    print("Yaw (Z-axis): {:.2f} degrees".format(yaw_deg))
        

# Extrinsic from "Accel"	  To	  "Color" :
#  Rotation Matrix:
#    0.999868         0.0146397       -0.00711235    
#   -0.0146384        0.999893         0.000237211   
#    0.00711506      -0.000133066      0.999975
# Extrinsic from "Gyro"	  To	  "Color" :
#  Rotation Matrix:
#    0.999868         0.0146397       -0.00711235    
#   -0.0146384        0.999893         0.000237211   
#    0.00711506      -0.000133066      0.999975

# V1 = np.array([9.969821131700451, -0.13084636315114604, -0.0044602660356624])
# V2 = np.array([9.832, -0.01299677531537071, -0.0017919685512529114])

# # 计算旋转矩阵
# theta = np.arccos(np.dot(V1, V2) / (np.linalg.norm(V1) * np.linalg.norm(V2)))
# k = np.cross(V1, V2)
# K = np.array([[0, -k[2], k[1]],
#               [k[2], 0, -k[0]],
#               [-k[1], k[0], 0]])
# R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)

# # 计算平移向量
# T = V2 - np.dot(R, V1)

# print("旋转矩阵:")
# print(R)
# euler_from_matrix(R)
# print("平移向量:")
# print(T)

# R = np.array([[0.00000132679, -1.000001, -0.00000132679],
#               [0.0000001, 0.000003267949, -1.000001],
#               [1.00000001, 0.0000013267949, 0.000000001]])
# euler_from_matrix(R)

# Rcl = np.array([[0.00162756,-0.999991,0.00390957,],
#                 [-0.0126748,-0.00392989,-0.999912],
#                 [0.999918,0.00157786,-0.012681]])

# # 计算矩阵的逆
# Rcl_inv = np.linalg.inv(Rcl)

# print("原始矩阵:")
# print(Rcl)
# print("\n逆矩阵:")
# print(Rcl_inv)

Rcamera2imu = np.array([[0.999868,-0.0146384,0.00711506,],
                [0.0146397,0.999893,-0.000133066],
                [-0.00711235,0.000237211,0.999975]])
roll = 1.5747174
pitch = -1.57275181
yaw = 0.00595955753
Rx = np.array([[1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]])
Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]])
Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]])
Rimu2lidar = Rz.dot(Ry).dot(Rx)
print("\n imu2lidar:")
print(Rimu2lidar)

Rcamera2lidar = Rcamera2imu.dot(Rimu2lidar)
Rlidar2camera = np.linalg.inv(Rcamera2lidar)
print("\n camera2lidar:")
print(Rcamera2lidar)
print("\n lidar2camera:")
print(Rlidar2camera)
