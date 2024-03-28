import alto
import time

alto_6_axis = alto.Alto(6, com_port="COM10")

#for i in range(5):
#    alto_6_axis.Alto_Go_RX(1.0)
#    time.sleep(0.05)
#    while alto_6_axis.Alto_Get_Position_Flag() == False:
#        print()

print(alto_6_axis.Alto_Go_Joint(4, 5))

print(alto_6_axis.Alto_Go_Joint(1, 20))
