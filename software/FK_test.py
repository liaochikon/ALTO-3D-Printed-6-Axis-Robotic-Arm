import alto
import util
import time
import numpy as np
import matplotlib.pyplot as plt

Workspace = [-500, 500, -500, 500, 0, 1000]

alto_6_axis = alto.Alto(6, com_port="COM10")

fig, ax = plt.subplots()
ax = fig.add_subplot(projection = '3d')
step = 3200 * 4

ax.cla()
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim3d(Workspace[0], Workspace[1])
ax.set_ylim3d(Workspace[2], Workspace[3])
ax.set_zlim3d(Workspace[4], Workspace[5])
R, t = util.T_to_R_and_t(np.eye(4))
util.Draw_Origin(R, t, ax, 100)

#alto_6_axis.Stepper_Abs(1, 3200)
#alto_6_axis.Stepper_Abs(5, 3200)
#time.sleep(3)

x =  alto_6_axis.Alto_Get_TCP_X()
y =  alto_6_axis.Alto_Get_TCP_Y()
z =  alto_6_axis.Alto_Get_TCP_Z()
rx = alto_6_axis.Alto_Get_TCP_RX()
ry = alto_6_axis.Alto_Get_TCP_RY()
rz = alto_6_axis.Alto_Get_TCP_RZ()
print([x, y, z , rx, ry, rz])

j1 = alto_6_axis.Alto_Get_Joint_Angle(1)
j2 = alto_6_axis.Alto_Get_Joint_Angle(2)
j3 = alto_6_axis.Alto_Get_Joint_Angle(3)
j4 = alto_6_axis.Alto_Get_Joint_Angle(4)
j5 = alto_6_axis.Alto_Get_Joint_Angle(5)
j6 = alto_6_axis.Alto_Get_Joint_Angle(6)
print([j1, j2, j3, j4, j5, j6])

j1 = alto_6_axis.Alto_Get_Target_Joint_Angle(1)
j2 = alto_6_axis.Alto_Get_Target_Joint_Angle(2)
j3 = alto_6_axis.Alto_Get_Target_Joint_Angle(3)
j4 = alto_6_axis.Alto_Get_Target_Joint_Angle(4)
j5 = alto_6_axis.Alto_Get_Target_Joint_Angle(5)
j6 = alto_6_axis.Alto_Get_Target_Joint_Angle(6)
print([j1, j2, j3, j4, j5, j6])

T = util.pose_to_translation_matrix(x, y, z, rx, ry, rz)
R, t = util.T_to_R_and_t(T)
util.Draw_Origin(R, t, ax, 50)
plt.show()