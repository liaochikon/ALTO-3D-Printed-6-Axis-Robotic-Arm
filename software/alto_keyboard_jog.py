import alto
import time
from pynput import keyboard

alto_6_axis = alto.Alto(6, com_port="COM10")
alto_6_axis.Alto_Operate()
time.sleep(0.05)

alto_6_axis.Alto_Set_Mov(alto.MovP)
alto_6_axis.Alto_Set_Speed(255)
alto_6_axis.Alto_Set_Acc(50)

speed = 10
omega = 2

def on_press(key):
    try:
        if key.char == 's':
            alto_6_axis.Alto_Go_X(speed)
        if key.char == 'w':
            alto_6_axis.Alto_Go_X(-speed)
        if key.char == 'd':
            alto_6_axis.Alto_Go_Y(speed)
        if key.char == 'a':
            alto_6_axis.Alto_Go_Y(-speed)
        if key.char == 'f':
            alto_6_axis.Alto_Go_Z(speed)
        if key.char == 'c':
            alto_6_axis.Alto_Go_Z(-speed)

        if key.char == 'u':
            alto_6_axis.Alto_Go_RX(omega)
        if key.char == 'j':
            alto_6_axis.Alto_Go_RX(-omega)
        if key.char == 'i':
            alto_6_axis.Alto_Go_RY(omega)
        if key.char == 'k':
            alto_6_axis.Alto_Go_RY(-omega)
        if key.char == 'o':
            alto_6_axis.Alto_Go_RZ(omega)
        if key.char == 'l':
            alto_6_axis.Alto_Go_RZ(-omega)
    except AttributeError:
        print('special key {0} pressed'.format(key))
def on_release(key):
    try:
        if key == keyboard.Key.esc:
            # Stop listener
            return False

    except AttributeError:
        if key == keyboard.Key.esc:
            # Stop listener
            return False

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
