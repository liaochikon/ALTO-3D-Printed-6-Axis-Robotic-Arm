from scipy.spatial.transform import Rotation
import numpy as np
import math

def Rotation_Matrix_to_Quaternion(R):
    r = Rotation.from_matrix(R)
    return r.as_quat()

def Quaternion_to_Rotation_Matrix(q):
    r = Rotation.from_quat(q)
    return r.as_matrix()

def Rotation_Matrix_to_Rotation_Vector(R):
    r = Rotation.from_matrix(R)
    return r.as_rotvec()

def Rotation_Vector_to_Rotation_Matrix(v):
    r = Rotation.from_rotvec(v)
    return r.as_matrix()

def Euler_Angle_to_Rotation_Matrix(e):
    r = Rotation.from_euler('xyz', e, degrees=True)
    return r.as_matrix()

def Rotation_Matrix_to_Euler_Angle(R):
    r = Rotation.from_matrix(R)
    return r.as_euler('xyz', degrees=True)

def Draw_Origin(R, t, ax, scale = 1):
    r0 = R[:, 0].reshape(3) * scale
    r1 = R[:, 1].reshape(3) * scale
    r2 = R[:, 2].reshape(3) * scale
    
    ax.quiver(t[0], t[1], t[2], r0[0], r0[1], r0[2], color='r')
    ax.quiver(t[0], t[1], t[2], r1[0], r1[1], r1[2], color='g')
    ax.quiver(t[0], t[1], t[2], r2[0], r2[1], r2[2], color='b')

def R_and_t_to_T(R, t):
    T = np.hstack((R, t))
    T = np.vstack((T, [0, 0, 0, 1]))
    return T

def T_to_R_and_t(T):
    Rt = T[:3]
    R = Rt[:, :3]
    t = Rt[:, 3].reshape((-1, 1))
    return R, t

def rotate_x(T, ang):
    Rx = np.array([[1,                           0,                            0, 0],
                   [0, math.cos(math.radians(ang)), -math.sin(math.radians(ang)), 0],
                   [0, math.sin(math.radians(ang)),  math.cos(math.radians(ang)), 0],
                   [0,                           0,                            0, 1]])
    T = np.matmul(Rx, T)
    return T

def rotate_y(T, ang):
    Ry = np.array([[ math.cos(math.radians(ang)), 0, math.sin(math.radians(ang)), 0],
                   [                           0, 1,                           0, 0],
                   [-math.sin(math.radians(ang)), 0, math.cos(math.radians(ang)), 0],
                   [                           0, 0,                           0, 1]])
    T = np.matmul(Ry, T)
    return T

def rotate_z(T, ang):
    Rz = np.array([[math.cos(math.radians(ang)), -math.sin(math.radians(ang)), 0, 0],
                   [math.sin(math.radians(ang)),  math.cos(math.radians(ang)), 0, 0],
                   [                          0,                            0, 1, 0],
                   [                          0,                            0, 0, 1]])
    T = np.matmul(Rz, T)
    return T

def translate(T, x, y, z):
    Rz = np.array([[ 1, 0, 0, x],
                   [ 0, 1, 0, y],
                   [ 0, 0, 1, z],
                   [ 0, 0, 0, 1]])
    T = np.matmul(Rz, T)
    return T

def pose_to_translation_matrix(x, y, z, rx, ry, rz):
    T = np.eye(4)
    T = rotate_x(T, rx)
    T = rotate_y(T, ry)
    T = rotate_z(T, rz)
    T[0][3] = x
    T[1][3] = y
    T[2][3] = z
    return T

def translation_matrix_to_pose(T):
    rz = math.degrees(math.atan2(T[1][0] , T[0][0])) 
    ry = math.degrees(math.asin(-T[2][0]))
    rx = math.degrees(math.atan2(T[2][1] , T[2][2]))
    x = T[0][3]
    y = T[1][3]
    z = T[2][3]
    return x, y, z, rx, ry, rz