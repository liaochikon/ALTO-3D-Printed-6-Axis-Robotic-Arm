import alto

alto_6_axis = alto.Alto(6, com_port="COM10")
alto_6_axis.Alto_Home()
print("homing axis...")