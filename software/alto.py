import serial
from serial.tools import list_ports
import time
import numpy as np

ERROR_HEX = 0xEE
DONE_HEX = 0xDD

MovP = 0
#MovL = 1
Joint = 1

cmd_count = 0

def cmd_num():
    global cmd_count
    cmd_count += 1
    return cmd_count - 1

ALTO_STEPPER_ABS = cmd_num()
ALTO_STEPPER_JOG = cmd_num()
ALTO_STEPPER_STOP = cmd_num()
ALTO_STEPPER_SET_ZERO = cmd_num()
ALTO_STEPPER_SET_EN = cmd_num()
ALTO_STEPPER_GET_ANGLE = cmd_num()
ALTO_STEPPER_GET_STEP = cmd_num()
ALTO_STEPPER_GET_ERROR = cmd_num()
ALTO_STEPPER_GET_EN = cmd_num()
ALTO_STEPPER_GET_BLOCK = cmd_num()
ALTO_STEPPER_GET_SPEED = cmd_num()
ALTO_STEPPER_GET_ACC = cmd_num()
ALTO_STEPPER_GET_ISBUSY = cmd_num()
ALTO_STEPPER_SET_SPEED = cmd_num()
ALTO_STEPPER_SET_ACC = cmd_num()
ALTO_GET_MODE = cmd_num()
ALTO_GET_POSITION_FLAG = cmd_num()
ALTO_GET_BUSY_FLAG = cmd_num()
ALTO_GET_HOME_FLAG = cmd_num()
ALTO_GET_TCP_X = cmd_num()
ALTO_GET_TCP_Y = cmd_num()
ALTO_GET_TCP_Z = cmd_num()
ALTO_GET_TCP_RX = cmd_num()
ALTO_GET_TCP_RY = cmd_num()
ALTO_GET_TCP_RZ = cmd_num()
ALTO_GET_TARGET_TCP_X = cmd_num()
ALTO_GET_TARGET_TCP_Y = cmd_num()
ALTO_GET_TARGET_TCP_Z = cmd_num()
ALTO_GET_TARGET_TCP_RX = cmd_num()
ALTO_GET_TARGET_TCP_RY = cmd_num()
ALTO_GET_TARGET_TCP_RZ = cmd_num()
ALTO_GET_JOINT_ANGLE = cmd_num()
ALTO_GET_TARGET_JOINT_ANGLE = cmd_num()
ALTO_OPERATE = cmd_num()
ALTO_HOME = cmd_num()
ALTO_DISABLE = cmd_num()
ALTO_START = cmd_num()
ALTO_STOP = cmd_num()
ALTO_SET_JOINT_HOME_OFFSET = cmd_num()
ALTO_SET_JOINT_ANGLE = cmd_num()
ALTO_SET_COOR = cmd_num()
ALTO_SET_SPEED = cmd_num()
ALTO_SET_ACC = cmd_num()
ALTO_SET_MOV = cmd_num()
ALTO_SET_X = cmd_num()
ALTO_SET_Y = cmd_num()
ALTO_SET_Z = cmd_num()
ALTO_SET_RX = cmd_num()
ALTO_SET_RY = cmd_num()
ALTO_SET_RZ = cmd_num()
ALTO_GO_X = cmd_num()
ALTO_GO_Y = cmd_num()
ALTO_GO_Z = cmd_num()
ALTO_GO_RX = cmd_num()
ALTO_GO_RY = cmd_num()
ALTO_GO_RZ = cmd_num()
ALTO_GO_JOINT = cmd_num()
ALTO_GET_MESSAGE = cmd_num()

def Find_Device(VID_PID = "0483:5740"):
    enmu_ports = enumerate(list_ports.comports())
    for n, (p, descriptor, hid) in enmu_ports:
        if hid.find(VID_PID) >= 0:
            print("Find device at " + p)
            print("Info : " + hid)
            return p
    print("ERROR : Can't find Alto Robot device in any COM port.")
    return None
    
def int32_to_uint8_array(i):
    i1 = np.uint8(i >> 24)
    i2 = np.uint8(i >> 16)
    i3 = np.uint8(i >> 8)
    i4 = np.uint8(i >> 0)
    return i1, i2, i3, i4

def int16_to_uint8_array(i):
    i1 = np.uint8(i >> 8)
    i2 = np.uint8(i >> 0)
    return i1, i2

def uint8_array_to_int32(i1, i2, i3, i4):
    i = np.int32((np.int32(i1) << 24) | 
                 (np.int32(i2) << 16) | 
                 (np.int32(i3) << 8) | 
                 (np.int32(i4) << 0))
    return i

def uint8_array_to_uint16(i1, i2):
    ui = np.uint16((np.uint16(i1) << 8) | 
                   (np.uint16(i2) << 0))
    return ui

def uint8_array_to_float32(i1, i2, i3, i4):
    m_int = np.int16((np.int16(i1) << 8) | 
                     (np.int16(i2) << 0))
    m_dec = np.int16((np.int16(i3) << 8) | 
                     (np.int16(i4) << 0))
    return m_int + m_dec / 10000

def float32_to_uint8_array(m):
    m_int = int(m)
    m_dec = int((m - m_int) * 10000)
    i1 = np.uint8(m_int >> 8)
    i2 = np.uint8(m_int >> 0)
    i3 = np.uint8(m_dec >> 8)
    i4 = np.uint8(m_dec >> 0)
    return i1, i2, i3, i4

class Alto():
    def __init__(self, axis_num, com_port = None, VID_PID = "0483:5740", baud_rate = 115200):
        if(com_port is None):
            com_port = Find_Device(VID_PID = VID_PID)
        self.COM_PORT = com_port
        self.BAUD_RATES = baud_rate
        self.Serial = serial.Serial(self.COM_PORT, self.BAUD_RATES)

        self.Axis_Num = axis_num

    def Receive(self):
        data_raw = self.Serial.read_all()
        return data_raw

    def Transmit(self, cmd):
        self.Serial.write(cmd)

    def Kill(self):
        self.Serial.close()

    def get_set_msg(self, c, wait_sec):
        delta_sec = 0
        start_sec = time.time()
        while delta_sec < wait_sec:
            delta_sec = time.time() - start_sec
            data_raw = self.Receive()
            if data_raw == b'':
                continue
            if data_raw[0] != c:
                continue
            if data_raw[1] == ERROR_HEX:
                return False
            if data_raw[1] == DONE_HEX:
                return True
        return False
    
    def get_int32(self, c, wait_sec, id = -1):
        delta_sec = 0
        start_sec = time.time()
        while delta_sec < wait_sec:
            delta_sec = time.time() - start_sec
            data_raw = self.Receive()
            if data_raw == b'':
                continue
            if data_raw[0] != c:
                continue
            if id > 0:#Message includes id 
                if data_raw[1] != id:
                    continue
                i = uint8_array_to_int32(data_raw[2] ,data_raw[3] ,data_raw[4] ,data_raw[5])
            else:#Message doesn't include id 
                i = uint8_array_to_int32(data_raw[1] ,data_raw[2] ,data_raw[3] ,data_raw[4])
            return i
        return None
    
    def get_uint16(self, c, wait_sec, id = -1):
        delta_sec = 0
        start_sec = time.time()
        while delta_sec < wait_sec:
            delta_sec = time.time() - start_sec
            data_raw = self.Receive()
            if data_raw == b'':
                continue
            if data_raw[0] != c:
                continue
            if id > 0:#Message includes id 
                if data_raw[1] != id:
                    continue
                i = uint8_array_to_uint16(data_raw[2] ,data_raw[3])
            else:#Message doesn't include id 
                i = uint8_array_to_uint16(data_raw[1] ,data_raw[2])
            return i
        return None
    
    def get_uint8(self, c, wait_sec, id = -1):
        delta_sec = 0
        start_sec = time.time()
        while delta_sec < wait_sec:
            delta_sec = time.time() - start_sec
            data_raw = self.Receive()
            if data_raw == b'':
                continue
            if data_raw[0] != c:
                continue
            if id > 0:#Message includes id 
                if data_raw[1] != id:
                    continue
                i = data_raw[2]
            else:#Message doesn't include id 
                i = data_raw[1]
            return i
        return None
        
    def get_float32(self, c, wait_sec, id = -1):
        delta_sec = 0
        start_sec = time.time()
        while delta_sec < wait_sec:
            delta_sec = time.time() - start_sec
            data_raw = self.Receive()
            if data_raw == b'':
                continue
            if data_raw[0] != c:
                continue
            if id > 0:#Message includes id 
                if data_raw[1] != id:
                    continue
                i = uint8_array_to_float32(data_raw[2] ,data_raw[3] ,data_raw[4] ,data_raw[5])
            else:#Message doesn't include id 
                i = uint8_array_to_float32(data_raw[1] ,data_raw[2] ,data_raw[3] ,data_raw[4])
            return i
        return None
    
    def get_bool(self, c, wait_sec, id = -1):
        delta_sec = 0
        start_sec = time.time()
        while delta_sec < wait_sec:
            delta_sec = time.time() - start_sec
            data_raw = self.Receive()
            if data_raw == b'':
                continue
            if data_raw[0] != c:
                continue
            if id > 0:#Message includes id 
                if data_raw[1] != id:
                    continue
                b = data_raw[2] == True
            else:#Message doesn't include id 
                b = data_raw[1] == True
            return b
        return None

    #Stepper cmd
    def Stepper_Abs(self, stepper_id, step, wait_sec = 0.05):
        self.Transmit([ALTO_STEPPER_ABS, stepper_id, *int32_to_uint8_array(step)])
        print("Moving stepper {:d} for {:d} step(s) in abs.".format(stepper_id, step))
        return self.get_set_msg(ALTO_STEPPER_ABS, wait_sec)

    def Stepper_Jog(self, stepper_id, step, wait_sec = 0.05):
        self.Transmit([ALTO_STEPPER_JOG, stepper_id, *int32_to_uint8_array(step)])
        print("Moving stepper {:d} for {:d} step(s) in jog.".format(stepper_id, step))
        return self.get_set_msg(ALTO_STEPPER_JOG, wait_sec)

    def Stepper_Stop(self, stepper_id, wait_sec = 0.05):
        self.Transmit([ALTO_STEPPER_STOP, stepper_id])
        print("Stoping stepper {:d}.".format(stepper_id))
        return self.get_set_msg(ALTO_STEPPER_STOP, wait_sec)

    def Stepper_Set_Zero(self, stepper_id, wait_sec = 0.05):
        self.Transmit([ALTO_STEPPER_SET_ZERO, stepper_id])
        print("Setting stepper {:d} zero.".format(stepper_id))
        return self.get_set_msg(ALTO_STEPPER_SET_ZERO, wait_sec)

    def Stepper_Enable(self, stepper_id, en, wait_sec = 0.05):
        self.Transmit([ALTO_STEPPER_SET_EN, stepper_id, en])
        print("Setting stepper {:d} Enable {:d}.".format(stepper_id, en))
        return self.get_set_msg(ALTO_STEPPER_SET_EN, wait_sec)
        
    def Stepper_Get_Angle(self, stepper_id, wait_sec = 0.05):
        self.Transmit([ALTO_STEPPER_GET_ANGLE, stepper_id])
        print("Get stepper {:d} angle.".format(stepper_id))
        return self.get_float32(ALTO_STEPPER_GET_ANGLE, wait_sec, stepper_id)

    def Stepper_Get_Step(self, stepper_id, wait_sec = 0.05):
        self.Transmit([ALTO_STEPPER_GET_STEP, stepper_id])
        print("Get stepper {:d} steps.".format(stepper_id))
        return self.get_int32(ALTO_STEPPER_GET_STEP, wait_sec, stepper_id)

    def Stepper_Get_Error(self, stepper_id, wait_sec = 0.05):
        self.Transmit([ALTO_STEPPER_GET_ERROR, stepper_id])
        print("Get stepper {:d} error.".format(stepper_id))
        return self.get_float32(ALTO_STEPPER_GET_ERROR, wait_sec, stepper_id)

    def Stepper_Get_En(self, stepper_id, wait_sec = 0.05):
        self.Transmit([ALTO_STEPPER_GET_EN, stepper_id])
        print("Get stepper {:d} enable.".format(stepper_id))
        return self.get_bool(ALTO_STEPPER_GET_EN, wait_sec, stepper_id)

    def Stepper_Get_Block(self, stepper_id, wait_sec = 0.05):
        self.Transmit([ALTO_STEPPER_GET_BLOCK, stepper_id])
        print("Get stepper {:d} block.".format(stepper_id))
        return self.get_bool(ALTO_STEPPER_GET_BLOCK, wait_sec, stepper_id)

    def Stepper_Get_Speed(self, stepper_id, wait_sec = 0.05):
        self.Transmit([ALTO_STEPPER_GET_SPEED, stepper_id])
        print("Get stepper {:d} speed.".format(stepper_id))
        return self.get_uint16(ALTO_STEPPER_GET_SPEED, wait_sec, stepper_id)
        
    def Stepper_Get_Acc(self, stepper_id, wait_sec = 0.005):
        self.Transmit([ALTO_STEPPER_GET_ACC, stepper_id])
        print("Get stepper {:d} acc.".format(stepper_id))
        return self.get_uint8(ALTO_STEPPER_GET_ACC, wait_sec, stepper_id)

    def Stepper_Get_IsBusy(self, stepper_id, wait_sec = 0.05):
        self.Transmit([ALTO_STEPPER_GET_ISBUSY, stepper_id])
        print("Get stepper {:d} busy.".format(stepper_id))
        return self.get_bool(ALTO_STEPPER_GET_ISBUSY, wait_sec, stepper_id)

    def Stepper_Set_Speed(self, stepper_id, speed, wait_sec = 0.05):        
        self.Transmit([ALTO_STEPPER_SET_SPEED, stepper_id, *int16_to_uint8_array(speed)])
        print("Set stepper {:d} speed to {:d}.".format(stepper_id, speed))
        return self.get_set_msg(ALTO_STEPPER_SET_SPEED, wait_sec)

    def Stepper_Set_Acc(self, stepper_id, acc, wait_sec = 0.05):
        self.Transmit([ALTO_STEPPER_SET_ACC, stepper_id, acc])
        print("Set stepper {:d} speed to {:d}.".format(stepper_id, acc))
        return self.get_set_msg(ALTO_STEPPER_SET_ACC, wait_sec)

    def Alto_Get_Mode(self, wait_sec = 0.05):
        self.Transmit([ALTO_GET_MODE])
        return self.get_uint8(ALTO_GET_MODE, wait_sec)
    
    def Alto_Get_Position_Flag(self, wait_sec = 0.05):
        self.Transmit([ALTO_GET_POSITION_FLAG])
        return self.get_uint8(ALTO_GET_POSITION_FLAG, wait_sec)
    
    def Alto_Get_Busy_Flag(self, wait_sec = 0.05):
        self.Transmit([ALTO_GET_BUSY_FLAG])
        return self.get_bool(ALTO_GET_BUSY_FLAG, wait_sec)
    
    def Alto_Get_Home_Flag(self, wait_sec = 0.05):
        self.Transmit([ALTO_GET_HOME_FLAG])
        return self.get_bool(ALTO_GET_HOME_FLAG, wait_sec)

    def Alto_Get_TCP_X(self, wait_sec = 0.05):        
        self.Transmit([ALTO_GET_TCP_X])
        print("Get TCP X.")
        return self.get_float32(ALTO_GET_TCP_X, wait_sec)

    def Alto_Get_TCP_Y(self, wait_sec = 0.05):        
        self.Transmit([ALTO_GET_TCP_Y])
        print("Get TCP Y.")
        return self.get_float32(ALTO_GET_TCP_Y, wait_sec)

    def Alto_Get_TCP_Z(self, wait_sec = 0.05):        
        self.Transmit([ALTO_GET_TCP_Z])
        print("Get TCP Z.")
        return self.get_float32(ALTO_GET_TCP_Z, wait_sec)

    def Alto_Get_TCP_RX(self, wait_sec = 0.05):        
        self.Transmit([ALTO_GET_TCP_RX])
        print("Get TCP RX.")
        return self.get_float32(ALTO_GET_TCP_RX, wait_sec)

    def Alto_Get_TCP_RY(self, wait_sec = 0.05):        
        self.Transmit([ALTO_GET_TCP_RY])
        print("Get TCP RY.")
        return self.get_float32(ALTO_GET_TCP_RY, wait_sec)

    def Alto_Get_TCP_RZ(self, wait_sec = 0.05):        
        self.Transmit([ALTO_GET_TCP_RZ])
        print("Get TCP RZ.")
        return self.get_float32(ALTO_GET_TCP_RZ, wait_sec)

    def Alto_Get_Target_TCP_X(self, wait_sec = 0.05):        
        self.Transmit([ALTO_GET_TARGET_TCP_X])
        return self.get_float32(ALTO_GET_TARGET_TCP_X, wait_sec)

    def Alto_Get_Target_TCP_Y(self, wait_sec = 0.05):        
        self.Transmit([ALTO_GET_TARGET_TCP_Y])
        return self.get_float32(ALTO_GET_TARGET_TCP_Y, wait_sec)

    def Alto_Get_Target_TCP_Z(self, wait_sec = 0.05):        
        self.Transmit([ALTO_GET_TARGET_TCP_Z])
        return self.get_float32(ALTO_GET_TARGET_TCP_Z, wait_sec)

    def Alto_Get_Target_TCP_RX(self, wait_sec = 0.05):        
        self.Transmit([ALTO_GET_TARGET_TCP_RX])
        return self.get_float32(ALTO_GET_TARGET_TCP_RX, wait_sec)

    def Alto_Get_Target_TCP_RY(self, wait_sec = 0.05):        
        self.Transmit([ALTO_GET_TARGET_TCP_RY])
        return self.get_float32(ALTO_GET_TARGET_TCP_RY, wait_sec)

    def Alto_Get_Target_TCP_RZ(self, wait_sec = 0.05):        
        self.Transmit([ALTO_GET_TARGET_TCP_RZ])
        return self.get_float32(ALTO_GET_TARGET_TCP_RZ, wait_sec)

    def Alto_Get_Joint_Angle(self, joint_id, wait_sec = 0.05):        
        self.Transmit([ALTO_GET_JOINT_ANGLE, joint_id])
        print("Get joint angle {:d}.".format(joint_id))
        return self.get_float32(ALTO_GET_JOINT_ANGLE, wait_sec, joint_id)
    
    def Alto_Get_Target_Joint_Angle(self, joint_id, wait_sec = 0.05):        
        self.Transmit([ALTO_GET_TARGET_JOINT_ANGLE, joint_id])
        print("Get joint angle {:d}.".format(joint_id))
        return self.get_float32(ALTO_GET_TARGET_JOINT_ANGLE, wait_sec, joint_id)

    def Alto_Operate(self, wait_sec = 0.05):
        self.Transmit([ALTO_OPERATE])
        return self.get_set_msg(ALTO_OPERATE, wait_sec)

    def Alto_Home(self, wait_sec = 0.05):
        self.Transmit([ALTO_HOME])
        return self.get_set_msg(ALTO_HOME, wait_sec)
    
    def Alto_Disable(self, wait_sec = 0.05):
        self.Transmit([ALTO_DISABLE])
        return self.get_set_msg(ALTO_DISABLE, wait_sec)

    def Alto_Start(self, wait_sec = 0.05):
        self.Transmit([ALTO_START])
        return self.get_set_msg(ALTO_START, wait_sec)
    
    def Alto_Stop(self, wait_sec = 0.05):
        self.Transmit([ALTO_STOP])
        return self.get_set_msg(ALTO_STOP, wait_sec)
    
    def Alto_Set_Joint_Home_Offset(self, wait_sec = 0.05):
        self.Transmit([ALTO_STOP])
        return self.get_set_msg(ALTO_STOP, wait_sec)
    
    def Alto_Set_Joint_Angle(self, joint_id, ang, wait_sec = 0.05):        
        self.Transmit([ALTO_SET_JOINT_ANGLE, joint_id, *float32_to_uint8_array(ang)])
        return self.get_set_msg(ALTO_SET_JOINT_ANGLE, wait_sec)

    def Alto_Set_Coor(self, coor, wait_sec = 0.05):
        self.Transmit([ALTO_SET_COOR, coor])
        return self.get_set_msg(ALTO_SET_COOR, wait_sec)
    
    def Alto_Set_Speed(self, speed, wait_sec = 0.05):
        self.Transmit([ALTO_SET_SPEED, speed])
        return self.get_set_msg(ALTO_SET_SPEED, wait_sec)
    
    def Alto_Set_Acc(self, acc, wait_sec = 0.05):
        self.Transmit([ALTO_SET_ACC, acc])
        return self.get_set_msg(ALTO_SET_ACC, wait_sec)

    def Alto_Set_Mov(self, mov, wait_sec = 0.05):
        self.Transmit([ALTO_SET_MOV, mov])
        return self.get_set_msg(ALTO_SET_MOV, wait_sec)

    def Alto_Set_X(self, x, wait_sec = 0.05):        
        self.Transmit([ALTO_SET_X, *float32_to_uint8_array(x)])
        return self.get_set_msg(ALTO_SET_X, wait_sec)
    
    def Alto_Set_Y(self, y, wait_sec = 0.05):        
        self.Transmit([ALTO_SET_Y, *float32_to_uint8_array(y)])
        return self.get_set_msg(ALTO_SET_Y, wait_sec)
    
    def Alto_Set_Z(self, z, wait_sec = 0.05):        
        self.Transmit([ALTO_SET_Z, *float32_to_uint8_array(z)])
        return self.get_set_msg(ALTO_SET_Z, wait_sec)
    
    def Alto_Set_RX(self, rx, wait_sec = 0.05):        
        self.Transmit([ALTO_SET_RX, *float32_to_uint8_array(rx)])
        return self.get_set_msg(ALTO_SET_RX, wait_sec)
    
    def Alto_Set_RY(self, ry, wait_sec = 0.05):        
        self.Transmit([ALTO_SET_RY, *float32_to_uint8_array(ry)])
        return self.get_set_msg(ALTO_SET_RY, wait_sec)
    
    def Alto_Set_RZ(self, rz, wait_sec = 0.05):        
        self.Transmit([ALTO_SET_RZ, *float32_to_uint8_array(rz)])
        return self.get_set_msg(ALTO_SET_RZ, wait_sec)
    
    def Alto_Go_X(self, x, wait_sec = 0.05):        
        self.Transmit([ALTO_GO_X, *float32_to_uint8_array(x)])
        return self.get_set_msg(ALTO_GO_X, wait_sec)
    
    def Alto_Go_Y(self, y, wait_sec = 0.05):        
        self.Transmit([ALTO_GO_Y, *float32_to_uint8_array(y)])
        return self.get_set_msg(ALTO_GO_Y, wait_sec)
    
    def Alto_Go_Z(self, z, wait_sec = 0.05):        
        self.Transmit([ALTO_GO_Z, *float32_to_uint8_array(z)])
        return self.get_set_msg(ALTO_GO_Z, wait_sec)
    
    def Alto_Go_RX(self, rx, wait_sec = 0.05):        
        self.Transmit([ALTO_GO_RX, *float32_to_uint8_array(rx)])
        return self.get_set_msg(ALTO_GO_RX, wait_sec)
    
    def Alto_Go_RY(self, ry, wait_sec = 0.05):        
        self.Transmit([ALTO_GO_RY, *float32_to_uint8_array(ry)])
        return self.get_set_msg(ALTO_GO_RY, wait_sec)
    
    def Alto_Go_RZ(self, rz, wait_sec = 0.05):        
        self.Transmit([ALTO_GO_RZ, *float32_to_uint8_array(rz)])
        return self.get_set_msg(ALTO_GO_RZ, wait_sec)
    
    def Alto_Go_Joint(self, joint_id, ang, wait_sec = 0.05):        
        self.Transmit([ALTO_GO_JOINT, joint_id, *float32_to_uint8_array(ang)])
        return self.get_set_msg(ALTO_GO_JOINT, wait_sec)