import alto
import time
from pynput import keyboard

alto_6_axis = alto.Alto(6, com_port="COM10")
points = []

alto_6_axis.Alto_Set_Speed(160)
alto_6_axis.Alto_Set_Acc(80)

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
    time.sleep(1)

def on_press(key):
    global points
    try:
        if key.char == 'o':
            alto_6_axis.Alto_Operate()
        if key.char == 'd':
            alto_6_axis.Alto_Disable()
        if key.char == 't':
            x = alto_6_axis.Alto_Get_TCP_X()
            x = alto_6_axis.Alto_Get_TCP_X()
            y = alto_6_axis.Alto_Get_TCP_Y()
            z = alto_6_axis.Alto_Get_TCP_Z()
            rx = alto_6_axis.Alto_Get_TCP_RX()
            ry = alto_6_axis.Alto_Get_TCP_RY()
            rz = alto_6_axis.Alto_Get_TCP_RZ()
            points.append([x, y, z , rx, ry, rz])
            print(points)
        if key.char == 'r':
            for i, p in enumerate(points):
                mov(p, str(i))
    except AttributeError:
        print('special key {0} pressed'.format(key))
    print("\n")

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False
    print("\n")

# Collect events until released
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()

# ...or, in a non-blocking fashion:
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()
