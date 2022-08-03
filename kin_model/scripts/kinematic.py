#!/usr/bin/env python

from glob import glob
from turtle import st
import rospy
import serial
import time
from geometry_msgs.msg import *
from std_msgs.msg import *
from pymodbus.client.sync import ModbusSerialClient
import crc16
import math
from geometry_msgs.msg import Pose

client = ModbusSerialClient(method='rtu', port = '/dev/RS485_232', stopbits = 1, parity = 'N', baudrate = 115200, strict=False)
ser = serial.Serial("/dev/RS485_232", 115200)

right_modBus_mode             = '01063A980002853C' #Fn_000 = 2 < Modbus bus mode
right_speedRun_mode           = '01063A9B0002753C' #Fn_003 = 2 < Speed operation
right_enable_hex              = '01063AA80001C532' #Fn_010 = 1 < Internally valid
right_workingAllowd_enable    = '010627200001B374' #Fn_013 = 1 < 

left_modBus_mode              = '02063A980002850F' #Fn_000 = 2
left_speedRun_mode            = '02063A9B0002750F' #Fn_003 = 2
left_enable_hex               = '02063AA80001C501' #Fn_010 = 1
left_workingAllowd_enable     = '020627230001B347' #Fn_013 = 1

def initialize():  
    if ser.is_open:
        print("port open success")
#------------------------------------------------------------------------
        byte_array1 = bytearray.fromhex(right_modBus_mode)
        ser.write(byte_array1)
        time.sleep(0.1)

        byte_array2 = bytearray.fromhex(right_speedRun_mode)
        ser.write(byte_array2)
        time.sleep(0.1)

        byte_array3 = bytearray.fromhex(right_enable_hex)
        ser.write(byte_array3)
        time.sleep(0.1)

        byte_array4 = bytearray.fromhex(right_workingAllowd_enable)
        ser.write(byte_array4)
        time.sleep(0.1)
#------------------------------------------------------------------------
        byte_array1 = bytearray.fromhex(left_modBus_mode)
        ser.write(byte_array1)
        time.sleep(0.1)

        byte_array2 = bytearray.fromhex(left_speedRun_mode)
        ser.write(byte_array2)
        time.sleep(0.1)

        byte_array3 = bytearray.fromhex(left_enable_hex)
        ser.write(byte_array3)
        time.sleep(0.1)

        byte_array4 = bytearray.fromhex(left_workingAllowd_enable)
        ser.write(byte_array4)
        time.sleep(0.1)
#------------------------------------------------------------------------     
    else:
        print("port open failed")

#---------------------------------------------------------------------------------------------
def hex2complement(number):

    global firstHex, secondHex
    numberOut = []

    bits = 16
    if number < 0:
        hexNum = hex((1 << bits) + number)[2:]
    else:
        hexNum = hex(number)[2:]

    for i in hexNum.zfill(4): #devide 2 hex value
        numberOut.append(i)

    firstHexStr = str(numberOut[0]) + str(numberOut[1]) # first hex value
    secondHexStr = str(numberOut[2]) + str(numberOut[3]) #second hex value

    an_integer1 = int(firstHexStr, 16) #conver to int
    an_integer2 = int(secondHexStr, 16) #conver to int

    firstHex = an_integer1 # for CRC calculator
    secondHex = an_integer2 # for CRC calculator

    return firstHexStr, secondHexStr, firstHex, secondHex

#---------------------------------------------------------------------------------------------
def crcCalculation(address, function_code, start_at_reg, num_of_reg):
    global crc_High, crc_Low
    hi_Append = []
    lo_Append = []

    loNum = []
    hiNum =[]

    read_device = address + function_code + start_at_reg + num_of_reg
    crc = crc16.calc(read_device)
    crc_hi = crc/256
    crc_lo = crc & 0xFF

    crc_Low = str(hex(crc_lo)[2:])
    crc_High = str(hex(crc_hi)[2:])

    for i in crc_Low.zfill(2): #devide 2 hex value
        loNum.append(i)

    for i in crc_High.zfill(2): #devide 2 hex value
        hiNum.append(i)

    crc_Low = str(loNum[0]) + str(loNum[1])
    crc_High = str(hiNum[0]) + str(hiNum[1])
    
    return crc_Low, crc_High

#---------------------------------------------------------------------------------------------
def wheel_Drive(leftSpeed, rightSpeed):
    global previousLeftData
    global previousRightData
    try:
        leftSpeed = int(leftSpeed)
        rightSpeed = int(rightSpeed)

        print(leftSpeed, rightSpeed)
        
        leftFirstHexStr, leftSecondHexStr, leftFirstHex, leftSecondHex = hex2complement(leftSpeed)
        leftAddress = chr(0x02)
        leftFunction_code = chr(0x06)
        leftStart_at_reg = chr(0x4E) + chr(0x20)
        leftNum_of_reg = chr(leftFirstHex) + chr(leftSecondHex)
        crc_Low, crc_High = crcCalculation(leftAddress, leftFunction_code, leftStart_at_reg, leftNum_of_reg)
        leftMotor = '02064E20' + str(leftFirstHexStr) + str(leftSecondHexStr) + str(crc_Low) + str(crc_High) #Add all hex code together Left Side

        rightFirstHexStr, rightSecondHexStr, rightFirstHex, rightSecondHex = hex2complement(rightSpeed)
        rightAddress = chr(0x01)
        rightFunction_code = chr(0x06)
        rightStart_at_reg = chr(0x4E) + chr(0x20)
        rightNum_of_reg = chr(rightFirstHex) + chr(rightSecondHex)
        rightCrc_Low, rightCrc_High = crcCalculation(rightAddress, rightFunction_code, rightStart_at_reg, rightNum_of_reg)
        rightMotor = '01064E20' + str(rightFirstHexStr) + str(rightSecondHexStr) + str(rightCrc_Low) + str(rightCrc_High) #Add all hex code together Right side
   
        if len(rightMotor) & len(leftMotor) == 16:
            client.close()
            ser.open

            if ser.is_open:
                byte_array1 = bytearray.fromhex(leftMotor)
                ser.write(byte_array1)
                time.sleep(0.01)

                byte_array = bytearray.fromhex(rightMotor)
                ser.write(byte_array)
                time.sleep(0.01)
                ser.close
                
            else:
                print("port open failed")
        else:
            print("Register Hex Code Not Complete")

        if len(rightMotor) & len(leftMotor) == 16:
            
            client.close()
            ser.open
            if ser.is_open:

                byte_array1 = bytearray.fromhex(leftMotor)
                ser.write(byte_array1)
                time.sleep(0.01)

                byte_array = bytearray.fromhex(rightMotor)
                ser.write(byte_array)
                time.sleep(0.01)
                ser.close
                
            else:
                print("port open failed")
        else:
            print("Register Hex Code Not Complete")
        
    except:
        print("Wheel Drive Error")
        mainFunction()


#---------------------------------------------------------------------------------------------
RightDirection = ""
LeftDirection = ""

st_mod = False #stearing angle is 90 => True
rt_mod = False #stearing angle is 0 => True
d = 0.31 #Lenth between wheels
L = 1.14  #Length of the robot
wd = 0.15 #wheel Diameter
s_ang = 0.0 #stearing angle

def wheelRST(): #rest stright pose
    global s_ang
    global st_mod
    global rt_mod
    # print ('wheelRST')
    if s_ang > math.pi/2 :
        # print('anti clock')
        return (-0.03,0.03)
    elif s_ang < math.pi/2 :
        # print ('clock')
        return (0.03,-0.03)
        
    else:
        return (0,0)

def wheelROT(omg): #rest rotate pose
    global s_ang 
    global st_mod
    global rt_mod
    # print ('wheelROT')

    if s_ang > 0 and omg > 0:
        return (-0.03,0.03)
    elif s_ang < 0 and omg > 0:
        return (0.03,-0.03)
    elif s_ang > math.pi and omg < 0:
        return (-0.03,0.03)
    elif s_ang < math.pi and omg < 0:
        return (0.03,-0.03)
    else:
        return (0,0)

def stop(vel,omg):
    # print ('stop')
    global st_mod
    if st_mod != True:
        # print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        return wheelRST() 
    else:
        return (0,0) 
    # if st_mod == True:
    #     return (0,0)

def straight(vel,omg):
    # print ('straight')
    global st_mod
    if st_mod != True:
        return wheelRST()
    else:
        return (vel,vel) 
    # if st_mod == True:
    #     return (vel,vel)

def rotate(vel,omg):
    # print ('rotate')
    global rt_mod
    global L
    global d
    if rt_mod != True:
        return wheelROT(omg)
    if rt_mod == True:
        Vleft = (L-d/2)*omg
        Vright = (L+d/2)*omg
        return (Vleft,Vright) 

def normal(vel,omg):
    # print ('normal')
    global L
    global d
    OA = math.sqrt((vel/omg)**2+L**2)*(omg/abs(omg))
    # print('OA' , OA)
    if vel > 0:
        Vleft = omg*(OA-d/2)
        Vright = omg*(OA+d/2)
    elif vel < 0:
        Vleft = -1*omg*(OA+d/2)
        Vright = -1*omg*(OA-d/2)
    print ('left', Vleft, 'right', Vright)
    return (Vleft,Vright)

def steer_angle(data):
    global s_ang 
    global st_mod
    global rt_mod
    s_ang = math.pi/2 - data.orientation.z
    # print ('s_ang: ', math.degrees(s_ang), 'raw ang' , math.degrees(data.orientation.z))
    if (math.pi/2) - 0.05 < s_ang < (math.pi/2) + 0.05: #change the filter
        st_mod = True
    else:
        st_mod = False
    if (-0.05 < s_ang < 0.05) or (math.pi-0.05 < s_ang < math.pi+0.05):  #change the filter
        rt_mod = True
    else:
        rt_mod = False

def control(data):
    global wd
    LeftVAL = rospy.Publisher('/LeftKinSpeed', Float64, queue_size=1)
    RightVAL = rospy.Publisher('/RightKinSpeed', Float64, queue_size=1)
    x_vel = data.linear.x
    z_omg = data.angular.z 
    # print (x_vel,z_omg,'data')

    if x_vel == 0 and z_omg == 0:
        Lw,Rw = stop(x_vel,z_omg)
    elif x_vel != 0 and z_omg == 0:
        Lw,Rw = straight(x_vel,z_omg)
    elif x_vel == 0 and z_omg != 0:
        Lw,Rw = rotate(x_vel,z_omg)
    elif x_vel != 0 and z_omg != 0:
        Lw,Rw = normal(x_vel,z_omg)

    
    
    Lomg = Lw*2/wd
    Romg = Rw*2/wd

    Lrpm = Lomg * 60 / (2 * math.pi)
    Rrpm = Romg * 60 / (2 * math.pi)

    LFrpm = 27 * Lrpm # Gear Ratio 1:27
    RFrpm = 27 * Rrpm # Gear Ratio 1:27

    # print('Lomg',Lomg ,'Romg', Romg ,'s ang', math.degrees(s_ang) , 'st', st_mod, 'rt', rt_mod)
    LeftVAL.publish(Lrpm)
    RightVAL.publish(Rrpm)

    wheel_Drive(LFrpm, RFrpm)


#---------------------------------------------------------------------------------------------
Right_PreviosVal = 0
Right_CurrentVal = 0
Right_TotalStichCount = 0
Right_StichCountDeference = 0

FirstRound = ""

Left_PreviosVal = 0
Left_CurrentVal = 0
Left_TotalStichCount = 0
Left_StichCountDeference = 0

# client.connect()

def readData(event):
    try:
        global Right_PreviosVal
        global Right_CurrentVal
        global Right_TotalStichCount
        global Right_StichCountDeference

        global Left_PreviosVal
        global Left_CurrentVal
        global Left_TotalStichCount
        global Left_StichCountDeference

        global FirstRound

        pub3 = rospy.Publisher('/right_ticks', Int16, queue_size=10) #Publish topic Initialize
        pub4 = rospy.Publisher('/left_ticks', Int16, queue_size=10)

        client.connect()

        if FirstRound == "True":
            RightHlist = []
            RightData = client.read_input_registers(address=29, count=2, unit = 1)

            for i in RightData.registers:
                RightHlist.append(i)
            client.close()
            
            RightCountPer_qur = RightHlist[0]
            RightQuarter_counter = RightHlist[1]
            RightTotalQur_counter = 65536 # total tick count

            RightTotalCountAll = (RightCountPer_qur + (RightTotalQur_counter * RightQuarter_counter))
            RightTotalCountInt = RightTotalCountAll / 1000
            RightTotalCount = int(RightTotalCountInt)

            Right_PreviosVal = RightTotalCount
        #------------------------------------------------------------------------------------------------

            client.connect()

            LeftHlist = []
            LeftData = client.read_input_registers(address=29, count=2, unit = 2)

            for i in LeftData.registers:
                LeftHlist.append(i)
            client.close()
            
            LeftCountPer_qur = LeftHlist[0]
            LeftQuarter_counter = LeftHlist[1]
            LeftTotalQur_counter = 65536 # total tick count

            LeftTotalCountAll = (LeftCountPer_qur + (LeftTotalQur_counter * LeftQuarter_counter))
            LeftTotalCountInt = LeftTotalCountAll / 1000
            LeftTotalCount = int(LeftTotalCountInt)

            Left_PreviosVal = LeftTotalCount
            
            FirstRound = "False"

        else:
            client.connect()

            if client.connect():

                RightHlist = []
                RightData = client.read_input_registers(address=29, count=2, unit = 1)

                for i in RightData.registers:
                    RightHlist.append(i)
                client.close()
                
                RightCountPer_qur = RightHlist[0]
                RightQuarter_counter = RightHlist[1]
                RightTotalQur_counter = 65536

                RightTotalCountAll = (RightCountPer_qur + (RightTotalQur_counter * RightQuarter_counter))
                RightTotalCountInt = RightTotalCountAll / 1000
                RightTotalCount = int(RightTotalCountInt)

                Right_CurrentVal = RightTotalCount
                
                Right_StichCountDeference = Right_CurrentVal - Right_PreviosVal

                
                if Right_StichCountDeference > 10000:
                    Right_TotalStichCount = (RightTotalCount - 4294967)
                    # print("1st IF <<<<<<<<<<<<<<<<<<<<<")

                elif Right_StichCountDeference < -10000:
                    Right_TotalStichCount = (RightTotalCount + 4294967)
                    Right_TotalStichCount += Right_StichCountDeference
                    # print(">>>>>>>>>>>>>>>>>>>>>>>>>> 2st IF")

                else:
                    Right_TotalStichCount += Right_StichCountDeference
                    # print("Else >>>>>>>>>>>---------<<<<<<<<<<<<<<< IF")
                
                Right_PreviosVal = Right_CurrentVal

    #---------------------------------------------------------------------------
                client.connect()

                LeftHlist = []
                LeftData = client.read_input_registers(address=29, count=2, unit = 2)

                for i in LeftData.registers:
                    LeftHlist.append(i)
                client.close()
                
                LeftCountPer_qur = LeftHlist[0]
                LeftQuarter_counter = LeftHlist[1]
                LeftTotalQur_counter = 65536

                LeftTotalCountAll = (LeftCountPer_qur + (LeftTotalQur_counter * LeftQuarter_counter))
                LeftTotalCountInt = LeftTotalCountAll / 1000
                LeftTotalCount = int(LeftTotalCountInt)

                Left_CurrentVal = LeftTotalCount
                
                Left_StichCountDeference = Left_CurrentVal - Left_PreviosVal
                
                
                if Left_StichCountDeference > 10000:
                    Left_TotalStichCount = (LeftTotalCount - 4294967)

                elif Left_StichCountDeference < -10000:
                    Left_TotalStichCount = (LeftTotalCount + 4294967)
                    Left_TotalStichCount += Left_StichCountDeference
                else:
                    Left_TotalStichCount += Left_StichCountDeference

                
                Left_PreviosVal = Left_CurrentVal

                pub3.publish(Right_TotalStichCount) #publish data to topic
                pub4.publish(Left_TotalStichCount)
                    
                # print(Right_TotalStichCount, Left_TotalStichCount)     #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

            else:
                print("No serial connection")
    except:
        print("Encoder Read Data Error")
        # mainFunction()
        client.connect()
        timer = rospy.Timer(rospy.Duration(0.0001), readData)
        timer.shutdown()
#---------------------------------------------------------------------------------------------
def mainFunction(): 
    rospy.init_node('kin_model', anonymous=True)

    while not rospy.is_shutdown(): 
        # client.connect()
        rospy.Subscriber('/wheel_base_pose', Pose ,steer_angle)
        rospy.Subscriber('/cmd_velue', Twist, control) #Subscribe /cmd_val topic
        # timer = rospy.Timer(rospy.Duration(0.0001), readData)
        rospy.spin()
        # timer.shutdown()

if __name__ == '__main__': 
    try:
        initialize()
        FirstRound = "True"
        mainFunction()

    except rospy.ROSInterruptException:
        pass