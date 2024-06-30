from __future__ import division
import rospy

from math import radians, pi

from std_msgs.msg import Float64

from myexception import *
from rtm_usb import *
import numpy as np



def getEncOffset_RsPs(DevNr):
    """
    function [ offset_1, offset_2 ] = getEncOffset_RsPs()
       calculate offsets of 2 encoders on the shoulder-board by reading encoders 10 times
    """

    # S12 board: DevNr = 0

    # init nodig in Python?
    # 1: pitch, 2: yaw
    #  moet eigen gegeven types zijn, maar dit lijkt minder te crashen.....
    #ADC = np.empty(7, dtype='uint16')
    ADC = np.empty(7)
    #Enc1, Enc2 = 0,0
    #Enc = np.empty(2, dtype='int32') 
    Enc = np.empty(2) 

    EncX = np.empty(10)
    EncY = np.empty(10)

    # namen hier eigenlijk ADC_parArr, ADC_perArr
    ADC_yawArr   = np.empty(10)
    ADC_pitchArr = np.empty(10)
    Enc_arr      = np.empty([2, 10])

    # Ignore the first pass as it contains old data -> cpl: why?
    (success, ADC, Enc ) = rtm_usb_ReadStdMsg(DevNr)
    if success != 0:
        raise RTMError('getEncOffset error: rtm_usb_ReadStdMsg')
    rtm_usb_Read_ready(DevNr)
    # read 10 times to average the result 
    success = rtm_usb_SendStdMsg(DevNr, 0, 0, 0, 0)
    if success != 0:
        raise RTMError('getEncOffset error: rtm_usb_SendStdMsg')
    for i in range(10):
        (success, ADC, Enc) = rtm_usb_ReadStdMsg(DevNr)
        if success != 0:
            raise RTMError('getEncOffset error: 2-nd rtm_usb_ReadStdMsg')
        # why the floats?
        EncX[i]  = float(Enc[0])
        EncY[i]  = float(Enc[1])
        # To complete the read-write cycle
        #rospy.sleep(0.5)
        rtm_usb_Read_ready(DevNr)
        success = rtm_usb_SendStdMsg(DevNr, 0, 0, 0, 0)
        if success != 0:
            raise RTMError('getEncOffset error: 3-rd rtm_usb_SendStdMsg')
        # wrist:
        ADC_pitchArr[i] = float(ADC[3]) # roll
        ADC_yawArr[i]   = float(ADC[4]) # pitch
    
    # nu goede naamgeving
    perMsVal = np.mean(ADC_pitchArr)
    parMsVal = np.mean(ADC_yawArr)

    #print 'Perpendicular value:', perMsVal
    #print 'Parallel value:',   parMsVal

    # hier een case specific case statement
    # S12 specific,
    perMsAngle = 90 - ((perMsVal - 35) / 2.0)        # 35 is the value at 90deg perpedicular to the table
    if parMsVal >= 528:
        parMsAngle = 90 - ((parMsVal - 528) / 2.0)   # 528 is the value at 90deg parallel to the table
    else:
        parMsAngle = -90 + ((167 - parMsVal) / 2.0)  # 167 is the value at -90deg parallel to the table


    #print 'perpedicular ms angle', perMsAngle
    #print 'parrallel  ms angle', parMsAngle

    # shoulder specific. Convert to?
    #EncTheta1 = pitchMsAngle / (2.43049633*(10**-3))    WERKT NIET!!
    EncTheta1 = (3.77 + perMsAngle)/(6.367308821541894*(0.0001)) - 5672.000000238419
    EncTheta2 = parMsAngle/(6.341209938079804*(0.0001))


    # omgekeerd!
    EncX_virt = EncTheta2 - EncTheta1
    EncY_virt = EncTheta1 + EncTheta2

    # waarom hier geen mean?
    offset_1 = EncX_virt - EncX[9]
    offset_2 = EncY_virt - EncY[9]

    # print 'S(yaw)-enc1 (analog, enc, angle, offset): ', perMsVal, EncX[9], perMsAngle, offset_1
    # print 'S(pit)-enc2 (analog, enc, angle, offset): ', parMsVal, EncY[9], parMsAngle, offset_2

    return offset_1, offset_2, perMsAngle*pi/180, parMsAngle*pi/180

def getEncOffset_PeYe(DevNr):
    """
    function [ offset_1, offset_2 ] = getEncOffset_PeYe()
       calculate offsets of 2 encoders on the elbow-board by reading encoders 10 times
    """

    # elbow board: DevNr = 1

    # init nodig in Python?
    # 1: pitch, 2: yaw
    #ADC1, ADC2 = 0,0
    #ADC = np.empty(7, dtype='uint16')
    ADC = np.empty(7)
    #Enc1, Enc2 = 0,0
    #Enc = np.empty(2, dtype='int32') 
    Enc = np.empty(2) 

    EncX = np.empty(10)
    EncY = np.empty(10)

    ADC_yawArr   = np.empty(10)
    ADC_pitchArr = np.empty(10)
    Enc_arr      = np.empty([2, 10])

    # Ignore the first pass as it contains old data -> cpl: why?
    (success, ADC, Enc ) = rtm_usb_ReadStdMsg(DevNr)
    if success != 0:
        raise RTMError('getEncOffset error: rtm_usb_ReadStdMsg')
    rtm_usb_Read_ready(DevNr)
    # read 10 times to average the result 
    success = rtm_usb_SendStdMsg(DevNr, 0, 0, 0, 0)
    if success != 0:
        raise RTMError('getEncOffset error: rtm_usb_SendStdMsg')
    for i in range(10):
        (success, ADC, Enc) = rtm_usb_ReadStdMsg(DevNr)
        if success != 0:
            raise RTMError('getEncOffset error: 2-nd rtm_usb_ReadStdMsg')
        # why the floats?
        EncX[i]  = float(Enc[0])
        EncY[i]  = float(Enc[1])
        # To complete the read-write cycle
        #rospy.sleep(0.5)
        rtm_usb_Read_ready(DevNr)
        success = rtm_usb_SendStdMsg(DevNr, 0, 0, 0, 0)
        if success != 0:
            raise RTMError('getEncOffset error: 3-rd rtm_usb_SendStdMsg')
        # wrist:
        ADC_pitchArr[i] = float(ADC[3])
        ADC_yawArr[i]   = float(ADC[4])
    
    pitchMsVal = np.mean(ADC_pitchArr)
    yawMsVal   = np.mean(ADC_yawArr)

    # hier een case specific case statement(English: here is a case specific case statement)
    # elbow specific
    if pitchMsVal <= 720 and pitchMsVal > 150:
        pitchMsAngle = (pitchMsVal - 574) / 2 - 1.71   # 574 is the value when the angle between the upper and lower arm is 180 deg -> added -1.71 to compensate for small offset, see notes
    else:
        pitchMsAngle = 147.5 - ((149 - pitchMsVal)/2.0)  # 73: number of degrees at the flip point

    yawMsVal = np.mean(ADC_yawArr)
    # remove jump in sensor reading
    if yawMsVal > 700:
        yawMsVal -= 713

    yawMsAngle = (yawMsVal - 213) / 2.0                 # 528 is the value at 90deg parallel to the table

    # elbow specific. Convert to?
    EncTheta1 = pitchMsAngle / (4.53855424*(0.0001));
    EncTheta2 = yawMsAngle / (4.38427091*(0.0001));


    EncX_virt = EncTheta1 - EncTheta2
    EncY_virt = EncTheta1 + EncTheta2

    offset_1 = EncX_virt - np.mean(EncX)
    offset_2 = EncY_virt - np.mean(EncY)

    # print 'E(pit)-enc1 (analog, enc, angle, offset): ', pitchMsVal, np.mean(EncX), pitchMsAngle, offset_1
    # print 'E(rol)-enc2 (analog, enc, angle, offset): ', yawMsVal, np.mean(EncY), yawMsAngle, offset_2

    return offset_1, offset_2, pitchMsAngle*pi/180, yawMsAngle*pi/180



def getEncOffset_S3Gr(DevNr):

    # S3Gripper board: DevNr = 2

    # init nodig in Python?
    # 1: pitch, 2: yaw
    #ADC1, ADC2 = 0,0
    #ADC = np.empty(7, dtype='uint16')
    ADC = np.empty(7)
    #Enc1, Enc2 = 0,0
    #Enc = np.empty(2, dtype='int32') 
    Enc = np.empty(2) 

    EncX = np.empty(10)
    EncY = np.empty(10)

    ADC_pitchArr = np.empty(10)
    Enc_arr      = np.empty([2, 10])

    # Ignore the first pass as it contains old data -> cpl: why?
    (success, ADC, Enc ) = rtm_usb_ReadStdMsg(DevNr)
    if success != 0:
        raise RTMError('getEncOffset error: rtm_usb_ReadStdMsg')
    rtm_usb_Read_ready(DevNr)
    # read 10 times to average the result 
    success = rtm_usb_SendStdMsg(DevNr, 0, 0, 0, 0)
    if success != 0:
        raise RTMError('getEncOffset error: rtm_usb_SendStdMsg')
    for i in range(10):
        (success, ADC, Enc) = rtm_usb_ReadStdMsg(DevNr)
        if success != 0:
            raise RTMError('getEncOffset error: 2-nd rtm_usb_ReadStdMsg')
        # why the floats?
        EncX[i]  = float(Enc[0])
        EncY[i]  = float(Enc[1])
        # To complete the read-write cycle
        #rospy.sleep(0.5)
        rtm_usb_Read_ready(DevNr)
        success = rtm_usb_SendStdMsg(DevNr, 0, 0, 0, 0)
        if success != 0:
            raise RTMError('getEncOffset error: 3-rd rtm_usb_SendStdMsg')
        # wrist:
        ADC_pitchArr[i] = float(ADC[3])
    
    # S3 controller

    # absolute motor positions
    pitchMsVal = np.mean(ADC_pitchArr)

    # absolute          -90     ...    +90
    #                424 --> 699 , 0 --> 74 
    #                424    .. y ..     773  (center 598.5)
    # encoder        e      .. x ..      e+190400
    #    absolute is y (pitchMsVal), encoder value is x
    if pitchMsVal < 100:
        pitchMsVal += 699.1

    # From 424 ..  773 analog to -90 to +90 degrees
    pitchMsAngle = (pitchMsVal - 598.5) / ((773-424)/180) 

    EncTheta1 = pitchMsAngle * (190400/180)

    # 
    offset_1 = - (np.mean(EncX) - EncTheta1)

    # old
    # absolute motor positions in degrees
    #pitchMsAngle = 144.5 - ((534.0 - pitchMsVal) / 2)
    # Convert to encoder steps
    #EncTheta1 = pitchMsAngle / (2.43049633*(0.001))

    # offset from encoder (in terms of joints and not of motor??)
    #offset_1 = EncTheta1 - np.mean(EncX)

    # print 'S(rol)-enc  (analog, enc, angle, offset): ', pitchMsVal, np.mean(EncX), pitchMsAngle, offset_1


    # Gripper controller

    """ Do homing action to the "closed" position of the gripper, then go 2000 encoder steps higher.
        This is the closed position. We give it angle 0. See first part of each finger
        The open position is with 75000 encoder steps higher. This is angle 45 degrees. So 1 degree equals 1666 encoder steps
        We only use the gripper between 0 and 45 degrees.
    """

    # met een boolean dit wel of niet doen. We gebruiken de gripper lang niet altijd. In parameters opnemen
    homing_torque = -25000
    # home the gripper(it sometimes get stuck in the completely open position)

    # Somehow this does not work reliably!!
    rtm_usb_SendStdMsg(DevNr,0,0, homing_torque,0)
    rospy.sleep(1.5)

    # determine encoder value
    for i in range(10):
        (success, ADC, Enc) = rtm_usb_ReadStdMsg(DevNr)
        if success != 0:
            raise RTMError('getEncOffset error: 2-nd rtm_usb_ReadStdMsg')
        # why the floats?
        EncY[i]  = float(Enc[1])
        # To complete the read-write cycle
        #rospy.sleep(0.5)
        rtm_usb_Read_ready(DevNr)
        success = rtm_usb_SendStdMsg(DevNr, 0, 0, homing_torque, 0)
        if success != 0:
            raise RTMError('getEncOffset error: 3-rd rtm_usb_SendStdMsg')
    # remove torque
    rtm_usb_SendStdMsg(DevNr,0,0,0,0)

    # It now is at -1.5 degrees. The controller should keep it between 0 (closed) and 45 (open) degrees
    pos = np.mean(EncY)
    offset_2 = -(pos + 2000)

    # print 'Gripper encoder (enc, angle, offset):           ', np.mean(EncY), (pos + offset_2)/1666.0, offset_2

    return offset_1, offset_2, pitchMsAngle*pi/180, -1.5




def getEncOffset_PwYw(DevNr):
    """
    function [ offset_1, offset_2 ] = getEncOffset_PwYw()
       calculate offsets of 2 encoders on the wrist-board by reading encoders 10 times
    """

    # wrist board: DevNr = 3
    #offset_1 = 8661.0; offset_2 = 9029.9
    #return offset_1, offset_2

    # init nodig in Python?
    # 1: pitch, 2: yaw
    #ADC1, ADC2 = 0,0
    #ADC = np.empty(7, dtype='uint16')
    ADC = np.empty(7)
    #Enc1, Enc2 = 0,0
    #Enc = np.empty(2, dtype='int32') 
    Enc = np.empty(2) 

    EncX = np.empty(10)
    EncY = np.empty(10)

    ADC_yawArr   = np.empty(10)
    ADC_pitchArr = np.empty(10)
    Enc_arr      = np.empty([2, 10])

    # Ignore the first pass as it contains old data -> cpl: why?
    (success, ADC, Enc ) = rtm_usb_ReadStdMsg(DevNr)
    if success != 0:
        raise RTMError('getEncOffset error: rtm_usb_ReadStdMsg')
    rtm_usb_Read_ready(DevNr)
    # read 10 times to average the result 
    success = rtm_usb_SendStdMsg(DevNr, 0, 0, 0, 0)
    if success != 0:
        raise RTMError('getEncOffset error: rtm_usb_SendStdMsg')
    for i in range(10):
        (success, ADC, Enc) = rtm_usb_ReadStdMsg(DevNr)
        if success != 0:
            raise RTMError('getEncOffset error: 2-nd rtm_usb_ReadStdMsg')
        # why the floats?
        EncX[i]  = float(Enc[0])
        EncY[i]  = float(Enc[1])
        # To complete the read-write cycle
        #rospy.sleep(0.5)
        rtm_usb_Read_ready(DevNr)
        success = rtm_usb_SendStdMsg(DevNr, 0, 0, 0, 0)
        if success != 0:
            raise RTMError('getEncOffset error: 3-rd rtm_usb_SendStdMsg')
        # wrist:
        ADC_pitchArr[i] = float(ADC[3])
        ADC_yawArr[i]   = float(ADC[4])

    # absolute motor positions
    pitchMsVal = np.mean(ADC_pitchArr)
    yawMsVal   = np.mean(ADC_yawArr)

    # wrist specific.
    # absolute motor positions in degrees?
    pitchMsAngle = 54.5 - ((534.0 - pitchMsVal) / 2)
    # wrist specific. 528 is the value at 90deg parallel to the table
    yawMsAngle = 39.0 - ((yawMsVal - 79) / 2)

    # Convert to?
    EncTheta1 = pitchMsAngle / (2.43049633*(0.001))
    EncTheta2 = yawMsAngle / (-2.71964947*(0.001))

    # absolute angle of joints
    EncX_virt = EncTheta1 - EncTheta2
    EncY_virt = EncTheta1 + EncTheta2

    # offset from encoder (in terms of joints and not of motor??)
    offset_1 = EncX_virt - np.mean(EncX)
    offset_2 = EncY_virt - np.mean(EncY)

    # print 'W(pit)-enc1 (analog, enc, angle, offset): ', pitchMsVal, np.mean(EncX), pitchMsAngle, offset_1
    # print 'W(yaw)-enc2 (analog, enc, angle, offset): ', yawMsVal, np.mean(EncY), yawMsAngle, offset_2

    """
    print 'Pitch value:', pitchMsVal
    print 'Yaw value:',   yawMsVal

    print 'pitch ms angle', pitchMsAngle
    print 'yaw ms angle', yawMsAngle

    print 'EncX, EncY', np.mean(EncX), np.mean(EncY)

    print "offset_1, offset_2", offset_1, offset_2

    Added current setting in radians

    """
    return offset_1, offset_2, pitchMsAngle*pi/180, yawMsAngle*pi/180

