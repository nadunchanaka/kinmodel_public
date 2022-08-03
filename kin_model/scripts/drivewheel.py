#!/usr/bin/env python3

from cgitb import reset
import rospy
import math
import time
from geometry_msgs.msg import *
from std_msgs.msg import *
import minimalmodbus
from geometry_msgs.msg import Pose


rightWheel = minimalmodbus.Instrument('/dev/RS485_232', 1)
rightWheel.serial.baudrate = 115200
rightWheel.serial.timeout = 0.2
rightWheel.mode = minimalmodbus.MODE_RTU
rightWheel.clear_buffers_before_each_transaction = True

leftWheel = minimalmodbus.Instrument('/dev/RS485_232', 2)
leftWheel.serial.baudrate = 115200
leftWheel.serial.timeout = 0.2
leftWheel.mode = minimalmodbus.MODE_RTU
leftWheel.clear_buffers_before_each_transaction = True

def initialize():  

    AccelarationTime = 500
    DeaccelarationTime = 500

    # Registernumber, value, number of decimals for storage
    rightWheel.write_register(15000, 2, 0)  #Fn_000 = 2 < Modbus bus mode
    rightWheel.write_register(15003, 2, 0)  #Fn_003 = 2 < Speed operation
    rightWheel.write_register(15016, 1, 0)  #Fn_010 = 1 < Internally valid
    rightWheel.write_register(10019, 1, 0)  #Fn_013 = 1 < Workin enable
    rightWheel.write_register(10184, AccelarationTime, 0) #Fn_0B8 Accelaration time in  ms
    rightWheel.write_register(10185, DeaccelarationTime, 0) #Fn_0B9 Deaccelaration time in ms

    leftWheel.write_register(15000, 2, 0)  #Fn_000 = 2 < Modbus bus mode 
    leftWheel.write_register(15003, 2, 0)  #Fn_003 = 2 < Speed operation
    leftWheel.write_register(15016, 1, 0)  #Fn_010 = 1 < Internally valid
    leftWheel.write_register(10019, 1, 0)  #Fn_013 = 1 < Workin enable
    leftWheel.write_register(10184, AccelarationTime, 0) #Fn_0B8 Accelaration time in ms
    leftWheel.write_register(10185, DeaccelarationTime, 0) #Fn_0B9 Deaccelaration time in ms

#---------------------------------------------------------------------------------------------

st_mod = False #stearing angle is 90 => True
rt_mod1 = False #stearing angle is 0 => True
rt_mod2 = False
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
    global rt_mod1, rt_mod2 
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
        # print("huttaaaaaaaaaaaaaaaaaaaaaaaaaa")
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
    global rt_mod1, rt_mod2
    global L
    global d
    if omg > 0:
        if rt_mod1 != True:
            return wheelROT(omg)
        elif rt_mod1 == True:
            Vleft = (L-d/2)*omg
            Vright = (L+d/2)*omg
            return (Vleft,Vright)
        print("rt_mod1 = ", rt_mod1)
    if omg < 0:
        if rt_mod2 != True:
            return wheelROT(omg)
        elif rt_mod2 == True:
            Vleft = (L+d/2)*omg * -1
            Vright = (L-d/2)*omg * -1
            return (Vleft,Vright) 
        print("rt_mod2 = ", rt_mod2)

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
    # print ('left', Vleft, 'right', Vright)
    return (Vleft,Vright)

def steer_angle(data):
    global s_ang 
    global st_mod
    global rt_mod1, rt_mod2
    s_ang = math.pi/2 + data.orientation.z
    # print ('s_ang: ', math.degrees(s_ang), 'raw ang' , math.degrees(data.orientation.z))
    if (math.pi/2) - 0.05 < s_ang < (math.pi/2) + 0.05: #change the filter
        st_mod = True
    else:
        st_mod = False
    if (-0.05 < s_ang < 0.05):  #change the filter
        rt_mod1 = True
    elif (math.pi-0.05 < s_ang < math.pi+0.05):
        rt_mod2 = True
    else:
        rt_mod1 = False
        rt_mod2 = False

LFrpm = 0
RFrpm = 0


def control(data):

    global wd

    global LFrpm, RFrpm
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
    LeftVAL.publish(LFrpm)
    RightVAL.publish(RFrpm)

    # print(s_ang)

    # wheel_Drive(LFrpm, RFrpm)




RightWheelKinInt = 0
LeftWheelKinInt = 0
        
def kinematic(data):
    global readDataCall

    global RightWheelKinInt
    global LeftWheelKinInt

    WheelBase = 0.31 #in meter
    WheelRadius = 0.15 
    
    vel_x = data.linear.x 
    omega = data.angular.z

    RightWheelKin = ((2 * vel_x) + (omega * WheelBase)) / (2 * WheelRadius) #kinematic equation
    LeftWheelKin = ((2 * vel_x) - (omega * WheelBase)) / (2 * WheelRadius)

    RightWheelKinInt = int(RightWheelKin) 
    LeftWheelKinInt = int(LeftWheelKin)

    RightWheelKinInt = RightWheelKinInt * 100
    LeftWheelKinInt = LeftWheelKinInt * 100

    # print(RightWheelKinInt, LeftWheelKinInt)
    
#---------------------------------------------------------------------------------------------

def callFunction(event):
    readData()
    writeData(RFrpm, LFrpm)

FirstRound = True

Right_PreviosVal = 0
Right_CurrentVal = 0
Right_TotalStichCount = 0
Right_StichCountDeference = 0

Left_PreviosVal = 0
Left_CurrentVal = 0
Left_TotalStichCount = 0
Left_StichCountDeference = 0

def readData():
    global Right_PreviosVal, Right_CurrentVal, Right_TotalStichCount, Right_StichCountDeference
    global Left_PreviosVal, Left_CurrentVal, Left_TotalStichCount, Left_StichCountDeference
    global FirstRound

    publishRight = rospy.Publisher('/right_ticks', Int64, queue_size=10) #Publish topic Initialize
    publishLeft = rospy.Publisher('/left_ticks', Int64, queue_size=10)

    if FirstRound == True:
        RightCountPer_qur = rightWheel.read_register(29, 0)  # Registernumber, number of decimals
        RightQuarter_counter = rightWheel.read_register(30, 0)
        RightTotalQur_counter = 65535
        RightTotalCountAll = (RightCountPer_qur + (RightTotalQur_counter * RightQuarter_counter))
        RightTotalCountInt = RightTotalCountAll / 100
        RightTotalCount = int(RightTotalCountInt)
        Right_PreviosVal = RightTotalCount

        LeftCountPer_qur = leftWheel.read_register(29, 0)  # Registernumber, number of decimals
        LeftQuarter_counter = leftWheel.read_register(30, 0)
        LeftTotalQur_counter = 65535
        LeftTotalCountAll = (LeftCountPer_qur + (LeftTotalQur_counter * LeftQuarter_counter))
        LeftTotalCountInt = LeftTotalCountAll / 100
        LeftTotalCount = int(LeftTotalCountInt)
        Left_PreviosVal = LeftTotalCount

        FirstRound = False
    else:
        RightCountPer_qur = rightWheel.read_register(29, 0)  # Registernumber, number of decimals
        RightQuarter_counter = rightWheel.read_register(30, 0)
        RightTotalQur_counter = 65535
        RightTotalCountAll = (RightCountPer_qur + (RightTotalQur_counter * RightQuarter_counter))
        RightTotalCountInt = RightTotalCountAll / 100
        RightTotalCount = int(RightTotalCountInt)
        Right_CurrentVal = RightTotalCount

        Right_StichCountDeference = Right_CurrentVal - Right_PreviosVal
        Right_TotalStichCount = Right_TotalStichCount + Right_StichCountDeference

        if Right_StichCountDeference > 10000:
            Right_TotalStichCount = Right_TotalStichCount - 42949017
        elif Right_StichCountDeference < -10000:
            Right_TotalStichCount = Right_TotalStichCount + 42949017

        publishRight.publish(Right_TotalStichCount)

        Right_PreviosVal = Right_CurrentVal

        LeftCountPer_qur = leftWheel.read_register(29, 0)  # Registernumber, number of decimals
        LeftQuarter_counter = leftWheel.read_register(30, 0)
        LeftTotalQur_counter = 65535
        LeftTotalCountAll = (LeftCountPer_qur + (LeftTotalQur_counter * LeftQuarter_counter))
        LeftTotalCountInt = LeftTotalCountAll / 100
        LeftTotalCount = int(LeftTotalCountInt)
        Left_CurrentVal = LeftTotalCount

        Left_StichCountDeference = Left_CurrentVal - Left_PreviosVal
        Left_TotalStichCount = Left_TotalStichCount + Left_StichCountDeference

        if Left_StichCountDeference > 10000:
            Left_TotalStichCount = Left_TotalStichCount - 42949017
        elif Left_StichCountDeference < -10000:
            Left_TotalStichCount = Left_TotalStichCount + 42949017

        publishLeft.publish(Left_TotalStichCount)

        Left_PreviosVal = Left_CurrentVal

    print(Right_TotalStichCount, Left_TotalStichCount)    

def writeData(RightWheelKinInt, LeftWheelKinInt):

    if RightWheelKinInt < 0 :
        rightRpm = 65536 + RightWheelKinInt
    else:
        rightRpm = RightWheelKinInt

    if LeftWheelKinInt < 0 :
        leftRpm = 65536 + LeftWheelKinInt
    else:
        leftRpm = LeftWheelKinInt

    leftWheel.write_register(20000, leftRpm, 0)
    rightWheel.write_register(20000, rightRpm, 0)


def mainFunction(): 
    rospy.init_node('kin_model', anonymous=True)

    while not rospy.is_shutdown(): 
        # writeData()
        rospy.Subscriber('/steer_pose', Pose ,steer_angle)
        rospy.Subscriber('/cmd_velue', Twist, control) #Subscribe /cmd_val topic
        # rospy.Subscriber('/cmd_velue', Twist, control) #Subscribe /cmd_val topic << from xz publisher for teleop keybord
        timer = rospy.Timer(rospy.Duration(0.001), callFunction)
        rospy.spin()
        timer.shutdown()

if __name__ == '__main__': 
    try:
        initialize()
        mainFunction()

    except rospy.ROSInterruptException:
        pass