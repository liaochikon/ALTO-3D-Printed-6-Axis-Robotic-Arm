import alto
import time

def mov(pose, name):
    alto_6_axis.Alto_Set_X(pose[0])
    alto_6_axis.Alto_Set_Y(pose[1])
    alto_6_axis.Alto_Set_Z(pose[2])
    alto_6_axis.Alto_Set_RX(pose[3])
    alto_6_axis.Alto_Set_RY(pose[4])
    alto_6_axis.Alto_Set_RZ(pose[5])
    alto_6_axis.Alto_Start()
    time.sleep(0.05)
    while alto_6_axis.Alto_Get_Position_Flag() == False:
        print(name + "\n")
        #os.system('cls')
    time.sleep(0.05)


alto_6_axis = alto.Alto(6, com_port="COM10")
alto_6_axis.Alto_Operate()
time.sleep(0.05)
alto_6_axis.Alto_Set_Mov(alto.MovP)

x = 360
y = 0
z = 310
rx = 90
ry = -90
rz = 90
alto_6_axis.Alto_Set_Speed(255)
alto_6_axis.Alto_Set_Acc(200)
#mov([x - 100, y , z + 100 , rx , ry, rz], "P1")
while True:
    mov([x - 100, y - 200, z + 160, rx, ry, rz], "P1")
    mov([x - 100, y - 200, z -  50, rx, ry, rz], "P2")
    mov([x - 100, y + 200, z -  50, rx, ry, rz], "P3")
    mov([x - 100, y + 200, z + 160, rx, ry, rz], "P4")

#for i in range(50000):
#    mov([x + i * 0.1, y  ,z , rx, ry, rz], str(i))