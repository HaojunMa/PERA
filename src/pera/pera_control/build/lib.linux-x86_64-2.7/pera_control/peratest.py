#!/usr/bin/env python
"""

Test program to control the  PERA

"""
import rospy

from math import radians

from std_msgs.msg import Float64
from diagnostic_msgs.msg import *

from rtmotion_usb_conf import *
from rtm_usb_SendStdMsg import *
from rtm_usb_ReadStdMsg import *
from getEncOffset_PwYw  import *

from myexception import *

def RTM2v_Init():
    """ Derived from the matlab version from Philips  """
    # Initialize RT-Motion USB architecture
    retval=rtm_usb_init()   
    print retval
    if retval != 0:
        print 'Failed to initialize RT-Motion USB architecture!'
        return
        # return an error and use it?

    AmpCh0 = 0;
    AmpCh1 = 1;
 
    #Displaying Version: Firmware, Board and CPLD
    for Devnr in range(4):
        st = 'unused'
        [res, st] = rtm_usb_Version(Devnr,st)                    # Display Firmware Version
        print 'Firmware Version: ', st
        [res, st ] = rtm_usb_BoardVersion(Devnr,st)              # Display Board Version
        print 'RT-Motion USB Board Version', ord(st[0])
        [res, st] = rtm_usb_CpldImageVersion(Devnr,st)           # Display Cpld Image Version
        print 'RT-Motion USB Onboard CPLD Image Version', ord(st[0])
        # Disable PID controller
        rtm_usb_Pid_DeleteAll(Devnr)
        rtm_usb_Pid_Dis(Devnr)



    # Sleeping mode and disable amplifiers before Tuning
    for Devnr in range(4):
       rtm_usb_Amp_Sleep(Devnr,AmpCh0,1)                # Puts the amplifier to sleeping mode
       rtm_usb_Amp_Sleep(Devnr,AmpCh1,1)                # Puts the amplifier to sleeping mode
       rtm_usb_Amp_Dis(Devnr,AmpCh0)                    # Disable amplifier
       rtm_usb_Amp_Dis(Devnr,AmpCh1)                    # Disable amplifier


    # Decay Mode: 2 and Blank Pin
    for Devnr in range(4):
        if (Devnr==0) or (Devnr==1):                    # Shoulder Motors (Board 0) or Elbow Motors (Board 1)
            rtm_usb_Amp_DecMode(Devnr,AmpCh0,2)         # 48% Decay Mode Channel X
            rtm_usb_Amp_DecMode(Devnr,AmpCh1,2)         # 48% Decay Mode Channel Y
            rtm_usb_Amp_Blank(Devnr,AmpCh0,1)           # Blank Pin = 1
            rtm_usb_Amp_Blank(Devnr,AmpCh1,1)           # Blank Pin = 1 
        if (Devnr==2):                                  # Shoulder Rotation Channel X and Gripper Channel Y
            rtm_usb_Amp_DecMode(Devnr,AmpCh0,1)         # Shoulder Rotation 18% Decay Mode
            rtm_usb_Amp_DecMode(Devnr,AmpCh1,1)         # Gripper 18% Decay Mode
            rtm_usb_Amp_Blank(Devnr,AmpCh0,1)           # Shoulder Rotation Blank Pin = 1
            rtm_usb_Amp_Blank(Devnr,AmpCh1,0)           # Gripper Blank Pin = 0
        if (Devnr==3):                                  # Wrist Motors (Board 3)
            rtm_usb_Amp_DecMode(Devnr,AmpCh0,0)         # Slow Decay Mode Channel X
            rtm_usb_Amp_DecMode(Devnr,AmpCh1,0)         # Slow Decay Mode Channel Y
            rtm_usb_Amp_Blank(Devnr,AmpCh0,0)           # Blank Pin = 0
            rtm_usb_Amp_Blank(Devnr,AmpCh1,0)           # Blank Pin = 0

    # External Mode
    for Devnr in range(4):
       rtm_usb_Amp_ExtMode(Devnr,AmpCh0,0)              # Channel X Set Amplifier ExtMode Pin (1: ON, 0: OFF)
       rtm_usb_Amp_ExtMode(Devnr,AmpCh1,0);             # Channel Y Set Amplifier ExtMode Pin (1: ON, 0: OFF)


    # Amplifier PWM 
    for Devnr in range(4):
        if (Devnr==0) or (Devnr==1) or (Devnr==2):       # Shoulder Motors (Board 0) or Elbow Motors (Board 1) or Shoulder Rotation Channel X and Gripper Channel Y
            rtm_usb_Amp_SetEnableSigPath(Devnr,AmpCh0,1) # Set Amplifier Enable Pin Physical Signal Path (1: Connected via PWM unit)
            rtm_usb_Amp_SetEnableSigPath(Devnr,AmpCh1,1) # Set Amplifier Enable Pin Physical Signal Path (1: Connected via PWM unit)
            rtm_usb_Amp_SetPwmSrc(Devnr,AmpCh0,2)        # Set Amplifier PWM Source (2: No PWM)
            rtm_usb_Amp_SetPwmSrc(Devnr,AmpCh1,2)        # Set Amplifier PWM Source (2: No PWM)
            rtm_usb_Amp_EnablePwm(Devnr,AmpCh0)          # Enable Amplifier PWM
            rtm_usb_Amp_EnablePwm(Devnr,AmpCh1)          # Enable Amplifier PWM
        if (Devnr==3):                                   # Wrist Motors (Board 3)
            rtm_usb_Amp_SetEnableSigPath(Devnr,AmpCh0,1) # Set Amplifier Enable Pin Physical Signal Path (1: Connected via PWM unit)
            rtm_usb_Amp_SetEnableSigPath(Devnr,AmpCh1,1) # Set Amplifier Enable Pin Physical Signal Path (1: Connected via PWM unit)
            rtm_usb_Amp_SetPwmThreshold(Devnr,AmpCh0,5000) # Set Amplifier PWM Treshhold Value: PWM Trashhold Value (max 0x7FFF)
            rtm_usb_Amp_SetPwmThreshold(Devnr,AmpCh1,5000) # Set Amplifier PWM Treshhold Value: PWM Trashhold Value (max 0x7FFF)
            rtm_usb_Amp_SetPwmThresholdGain(Devnr,AmpCh0,1,1) # Set Gain for Amplifier PWM Treshold Region (1: Amplifier Reference Multiplier and 1:Amplifier Reference Divider)
            rtm_usb_Amp_SetPwmThresholdGain(Devnr,AmpCh1,1,1) # Set Gain for Amplifier PWM Treshold Region (1: Amplifier Reference Multiplier and 1:Amplifier Reference Divider)
            rtm_usb_Amp_SetPwmSrc(Devnr,AmpCh0,1)        # Set Amplifier PWM Source 1: HW PWM
            rtm_usb_Amp_SetPwmSrc(Devnr,AmpCh1,1)        # Set Amplifier PWM Source 1: HW PWM
            rtm_usb_Amp_EnablePwm(Devnr,AmpCh0)          # Enable Amplifier PWM
            rtm_usb_Amp_EnablePwm(Devnr,AmpCh1)          # Enable Amplifier PWM


    #Thermal checking
    #   niet

    for Devnr in range(4):
        rtm_usb_RtmCount_SetSource(Devnr,2,0)            # Select HW Encoder Counter Source (2:Onboard RTM-Count with Parallel Interface is Selected, 0:Digital I/O port (only port 0 for board version 2))
        for AmpCh in range(2):
            rtm_usb_RtmCount_EnCh(Devnr,AmpCh)           # Enable RTM-Count Channel
            rtm_usb_RtmCount_ResetCh(Devnr,AmpCh)        # Reset RTM-Count Channel
        rtm_usb_RtmCount_EnCycRead(Devnr,1,3000)         # Enable cyclic reading (1: real-time and 3000: Real-time sampling period)
    
    #Enables A/D Convertion
    for Devnr in range(4):
        for ADCCh in range(7):
            rtm_usb_ADC_EN(Devnr,ADCCh)                   # Channels 0-4: 10-Bit ADC, Channels 5-6: 16-Bit ADC

    #Non-Sleeping mode and Enable amplifiers after Tuning
    for Devnr in range(4):
        for AmpCh in range (2):
            rtm_usb_Amp_EN(Devnr,AmpCh)                   # Enable amplifier
            rtm_usb_Amp_Sleep(Devnr,AmpCh,0)              # Puts the amplifier to Wake mode

    for Devnr in range(4):
        rtm_usb_DAC_EN(Devnr,AmpCh0)                      # Enables the D/A converter
        rtm_usb_DAC_EN(Devnr,AmpCh1)                      # Enables the D/A converter    

    #Initial position for Axis 0 degrees.
    for Devnr in range(4):
        rtm_usb_SendStdMsg(Devnr,0,0,0,0)               #Sets all axis in 0.


    # Enables Software Encoder Counting
    # niet


def controller():
    # de controller


    try:
        # trail run (first one is slow) (4 devices)
        #   wat gebeurd hier precies? 
        for Devnr in range(4):
            rtm_usb_ReadStdMsg(Devnr)
            rtm_usb_Dread_ready(Devnr);
            rtm_usb_SendStdMsg(Devnr,0, 0, 0, 0);
    except MyError as e:
        print 'My exception occurred, value:', e.value

    # from now on only the wrist board
    DevNr = 3
    # Get the encoder offsets for encoder X and Y of the wrist
    offset_6, offset_7 = getEncOffset_PwYw(DevNr)
    
    # the controller loop
    r = rospy.Rate(10)

    Enc = np.zeros(2, dtype='int32') 


    i = 0 # number of iterations
    running = True
    while running == True:
        print "Loop"
        # get encoder values from the wrist board
        #_, _, Enc = rtm_usb_ReadStdMsg(Devnr)
        EncWX, EncWY = float(Enc[0]), float(Enc[1])

        #  Set the encoders to zero, offset is generated from the hall effect sensors
        EncWX = EncWX + offset_6
        EncWY = EncWY + offset_7
    
        EncTheta6 = 0.5*(EncWX + EncWY)
        EncTheta7 = 0.5*(EncWY - EncWX)

        #  Convert encoder to degrees
        EncTheta6 = EncTheta6 * (2.43049633*10**-3);
        EncTheta7 = EncTheta7 * (-2.71964947*10**-3);
        # limit
        if EncTheta6 < -57:
            EncTheta6 = -57
        if EncTheta6 > 57:
            EncTheta6 = 57
        if EncTheta7 < -45:
            EncTheta7 = -45
        if EncTheta7 > 45:
            EncTheta7 = 45


        #  Computation of the speed using the Encoders

        #  Average speed using last 10 values

        #  Theta and dTheta in radians

        #  Calculating eTheta1(i) and first and second derivative in radians
        # e staat voor dEsired position

        #  Calculating eTheta2(i) and first and second derivative in radians

        #  call actual controller

        #  Send result to motors


        i = i+1
        if i/20 == 0:
            print 'iets'
            # toggle setpoint
        if i == 50:
            running = False
        # get time needed from beginning of this iteratioin

        # sleep untill next iteration
        r.sleep()

    # Do shutdown
    print 'Shutdown'

def testinit():
    n="namespace"

    # from parameter server
    id = int(rospy.get_param(n+"id", 10))
    neutral = rospy.get_param(n+"neutral", 0)

    now = rospy.Time.now()
    print("Current time %i %i", now.secs, now.nsecs)

    # ROS interfaces
    rospy.Subscriber(n+'/command', Float64, commandCb)

    w_delta = rospy.Duration(1.0/rospy.get_param("~write_rate", 50.0))
    w_next = rospy.Time.now() + w_delta

    RTM2v_Init()
    
    controller()

def commandCb(req):
    """ Float64 style command input. """
    desired = req.data


if __name__ == "__main__":
    rospy.init_node('peratestinit')
    testinit()

        
