import numpy as np


def matrix_to_quaternion(matrix):
    # 提取旋转部分
    rotation_matrix = matrix[:3, :3]

    # 计算四元数的各个分量
    trace = np.trace(rotation_matrix)
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * S
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / S
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / S
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / S
    elif (rotation_matrix[0, 0] > rotation_matrix[1, 1]) and (rotation_matrix[0, 0] > rotation_matrix[2, 2]):
        S = np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2.0
        qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / S
        qx = 0.25 * S
        qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
        qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
    elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
        S = np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2.0
        qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / S
        qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
        qy = 0.25 * S
        qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
    else:
        S = np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2.0
        qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / S
        qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
        qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
        qz = 0.25 * S

    return np.array([qx, qy, qz, qw])




# 将rosbag的pose数据再处理
def main():
    root_dir = "/media/pengll/SSD/data/FAST-LIVO_data/NTU VIRAL DATASET/viral_eval-master/result_nya_02/"
    input_file = root_dir + "predict_odom.csv"
    output_file = root_dir + "predict_odom_new.txt"
    lidar_pose_file = root_dir + "lidar_pose.txt"

    with open(input_file, "r") as f_input, open(output_file, "w") as f_output:
        i=0
        for line in f_input:
            if i<=9:
                i+=1
                continue
            # 切分每行数据，以逗号分隔
            parts = line.strip().split(',')
            # 去除前两列数据
            filtered_line = ' '.join(parts[2:])
            # 确保每个数据的长度不超过11
            for s in filtered_line.split():
                if len(s) > 12:
                    s = s[:11]
                f_output.write(str(s))
                f_output.write(' ')
            f_output.write('\n')
            
    # rosbag的数据是leica坐标系的，转化为lidar坐标系
    matrix_T_Body_Prism = np.array([
        [1.0, 0.0, 0.0, -0.293656],
        [0.0, 1.0, 0.0, -0.012288],
        [0.0, 0.0, 1.0, -0.273095],
        [0.0, 0.0, 0.0, 1.0]
    ])

    matrix_T_Body_Lidar = np.array([
        [1.0, 0.0, 0.0, -0.050],
        [0.0, 1.0, 0.0, 0.000],
        [0.0, 0.0, 1.0, 0.055],
        [0.0, 0.0, 0.0, 1.0]
    ])

    matrix_T_Body_Lidar_inverse = np.linalg.inv(matrix_T_Body_Lidar)
    matrix_T_Body_Prism_inverse = np.linalg.inv(matrix_T_Body_Prism)

    # 这个result 1 2 是测试哪种转换是正确的
    result1 = np.dot(matrix_T_Body_Lidar_inverse , matrix_T_Body_Prism)
    result2 = np.dot(matrix_T_Body_Lidar , matrix_T_Body_Prism_inverse)

    with open(output_file, "r") as f_output:
        lines = f_output.readlines()

    rotation_matrix = np.eye(3)
    with open(lidar_pose_file, "w") as f_output:
        i = 0
        for line in lines:
            parts = line.strip().split()
            if i==0:
                i+=1
                x_0 = float(parts[1])
                y_0 = float(parts[2])
                z_0 = float(parts[3])
                q_x = float(parts[4])
                q_y = float(parts[5])
                q_z = float(parts[6])
                q_w = float(parts[7])
                rotation_matrix = np.array([
                    [1 - 2*(q_y**2 + q_z**2), 2*(q_x*q_y - q_w*q_z), 2*(q_x*q_z + q_w*q_y), x_0],
                    [2*(q_x*q_y + q_w*q_z), 1 - 2*(q_x**2 + q_z**2), 2*(q_y*q_z - q_w*q_x), y_0],
                    [2*(q_x*q_z - q_w*q_y), 2*(q_y*q_z + q_w*q_x), 1 - 2*(q_x**2 + q_y**2), z_0],
                    [0.0,0.0,0.0,1.0]
                ])
                
            
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
            q_x = float(parts[4])
            q_y = float(parts[5])
            q_z = float(parts[6][:8])
            q_w = float(parts[7])
            
            # point = np.array([x, y, z, 1.0])
            # point_new = np.dot(result1, point)
            
            # 将初始时刻设置为原点
            rotation_matrix_inverse = np.linalg.inv(rotation_matrix)
            
            pose_old = np.array([
                [1 - 2*(q_y**2 + q_z**2), 2*(q_x*q_y - q_w*q_z), 2*(q_x*q_z + q_w*q_y), x],
                [2*(q_x*q_y + q_w*q_z), 1 - 2*(q_x**2 + q_z**2), 2*(q_y*q_z - q_w*q_x), y],
                [2*(q_x*q_z - q_w*q_y), 2*(q_y*q_z + q_w*q_x), 1 - 2*(q_x**2 + q_y**2), z],
                [0.0,0.0,0.0,1.0]
            ])
            
            pose_new = np.dot(rotation_matrix_inverse, pose_old)
            quaternion = matrix_to_quaternion(pose_new)
            
            f_output.write(str(parts[0]))
            f_output.write(' ')
            f_output.write(str(pose_new[0][3])[:11])
            f_output.write(' ')
            f_output.write(str(pose_new[1][3])[:11])
            f_output.write(' ')
            f_output.write(str(pose_new[2][3])[:11])
            f_output.write(' ')
            f_output.write(str(quaternion[0])[:11])  # q_x
            f_output.write(' ')
            f_output.write(str(quaternion[1])[:11])  # q_y
            f_output.write(' ')
            f_output.write(str(quaternion[2])[:11])  # q_z
            f_output.write(' ')
            f_output.write(str(quaternion[3])[:11])  # q_w
            f_output.write('\n')


# 坐标绕x y z轴旋转180度代码
def rot_X_Y_Z():
    root_path = "/home/pengll/nya_02"
    posefile = root_path + ".txt"
    posefile_X = root_path + "_X.txt"
    posefile_Y = root_path + "_Y.txt"
    posefile_Z = root_path + "_Z.txt"
    with open(posefile, "r") as f, open(posefile_X, "w") as f_output_X, open(posefile_Y, "w") as f_output_Y, open(posefile_Z, "w") as f_output_Z:
        lines = f.readlines()
        for line in lines:
            parts = line.strip().split()
            timestamp = parts[0]
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
            x_rotX = x
            y_rotX = -y
            z_rotX = -z
            f_output_X.write(str(x_rotX))
            f_output_X.write(' ')
            f_output_X.write(str(y_rotX))
            f_output_X.write(' ')
            f_output_X.write(str(z_rotX))
            f_output_X.write('\n')
            
            x_rotY = -x
            y_rotY = y
            z_rotY = -z
            f_output_Y.write(str(x_rotY))
            f_output_Y.write(' ')
            f_output_Y.write(str(y_rotY))
            f_output_Y.write(' ')
            f_output_Y.write(str(z_rotY))
            f_output_Y.write('\n')
            
            x_rotZ = -x
            y_rotZ = -y
            z_rotZ = z
            f_output_Z.write(str(x_rotZ))
            f_output_Z.write(' ')
            f_output_Z.write(str(y_rotZ))
            f_output_Z.write(' ')
            f_output_Z.write(str(z_rotZ))
            f_output_Z.write('\n')
            
# 写一个将fast-livo输出的pose和ntu_gt的pose按照时间戳进行匹配函数
# livo的pose起始时间早，结束时间晚，需要掐头去尾
def match_pose():
    livo_pose_file = "/home/pengll/data/pos_log_n02.txt"
    ntu_pose_file = "/home/pengll/data/nya_02.txt"
    livo_pose_file_output = "/home/pengll/data/pos_log_n02_matched.txt"
    ntu_pose_file_output = "/home/pengll/data/nya_02_matched.txt"
    
    livo_dict = {}
    ntu_dict = {}
    livo_times = []
    ntu_times = []
    with open(livo_pose_file, "r") as f_livo, open(ntu_pose_file, "r") as f_ntu, open(livo_pose_file_output, "w") as f_output_livo, open(ntu_pose_file_output, "w") as f_output_ntu:
        lines_livo = f_livo.readlines()
        for line_livo in lines_livo:
            parts_livo = line_livo.strip().split()
            pose = {'x':float(parts_livo[1]), 'y':float(parts_livo[2]), 'z':float(parts_livo[3])}
            livo_dict[float(parts_livo[0])] = pose
            livo_times.append(float(parts_livo[0]))
            
        lines_ntu = f_ntu.readlines()
        for line_ntu in lines_ntu:
            parts_ntu = line_ntu.strip().split()
            pose = {'x':float(parts_ntu[1]), 'y':float(parts_ntu[2]), 'z':float(parts_ntu[3])}
            ntu_dict[float(parts_ntu[0])*10] = pose
            ntu_times.append(float(parts_ntu[0])*10)
        ntu_times_np = np.array(ntu_times)
        for time in livo_times:
            time_diff = np.abs(ntu_times_np - time)
            min_index = np.argmin(time_diff)
            if time_diff[min_index] >= 1:
                continue
            f_output_livo.write("{:.0f}".format(time))
            f_output_livo.write(' ')
            f_output_livo.write("{:.6f}".format(livo_dict[time]['x']))
            f_output_livo.write(' ')
            f_output_livo.write("{:.6f}".format(livo_dict[time]['y']))
            f_output_livo.write(' ')
            f_output_livo.write("{:.6f}".format(livo_dict[time]['z']))
            f_output_livo.write(' ')
            f_output_livo.write(str(0))
            f_output_livo.write(' ')
            f_output_livo.write(str(0))
            f_output_livo.write(' ')
            f_output_livo.write(str(0))
            f_output_livo.write(' ')
            f_output_livo.write(str(1))
            f_output_livo.write('\n')
            
            f_output_ntu.write("{:.0f}".format(ntu_times[min_index]))
            f_output_ntu.write(' ')
            f_output_ntu.write("{:.6f}".format(ntu_dict[ntu_times[min_index]]['x']))
            f_output_ntu.write(' ')
            f_output_ntu.write("{:.6f}".format(ntu_dict[ntu_times[min_index]]['y']))
            f_output_ntu.write(' ')
            f_output_ntu.write("{:.6f}".format(ntu_dict[ntu_times[min_index]]['z']))
            f_output_ntu.write(' ')
            f_output_ntu.write(str(0))
            f_output_ntu.write(' ')
            f_output_ntu.write(str(0))
            f_output_ntu.write(' ')
            f_output_ntu.write(str(0))
            f_output_ntu.write(' ')
            f_output_ntu.write(str(1))
            f_output_ntu.write('\n')
            
