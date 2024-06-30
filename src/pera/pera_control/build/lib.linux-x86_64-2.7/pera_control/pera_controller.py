#!/usr/bin/env python

"""
   Controllers for pera
"""
import rospy

import numpy as np
from math import radians

from std_msgs.msg import Float64
from diagnostic_msgs.msg import *

from urdf_parser_py.urdf import URDF
from boards import *

# 3 controllers zijn vrijwel hetzelfde.
# met een klasse (voor de 3 dual controllers) waar bijv. de gebruikte __init__ in staat is het mogelijk nog wat compacter te maken.
#
# hier de verschillende dedicated controllers, de pid functie in een class ivm zetten van de parameters?

# template for the controller function proper
class PIDfunction():
    """
       PID function with parameters set
       inputs in radians
       output klaar voor de DAC
    """
    def __init__(self,P,I,D):
        print 'set pid variables'

    def output():
        return 0,0


#############################################
class S12controller(Board):
    """
       controller for S12 joints
           for differential drive
       contains all state of the board
    """
    def __init__(self,device, DevNr):
        Board.__init__(self, device)
        self.fake = device.fake
        self.rate = device.rate

        # parameters voor dit board ophalen
        # these are all constants
        joints = device.joints
        for x in joints:
            if joints[x].get('devnr') == DevNr:
                channr =  joints[x].get('channel')
                if channr == 0:
                    self.name0 = x
                    self.P0 = joints[x].get('P')
                    self.I0 = joints[x].get('I')
                    self.D0 = joints[x].get('D')
                    self.lower0 = joints[x].get('lower')
                    self.upper0 = joints[x].get('upper')
                    self.controller0 = joints[x].get('controller')
                else:
                    self.name1 = x
                    self.P1 = joints[x].get('P')
                    self.I1 = joints[x].get('I')
                    self.D1 = joints[x].get('D')
                    self.lower1 = joints[x].get('lower')
                    self.upper1 = joints[x].get('upper')
                    self.controller1 = joints[x].get('controller')

        # een controller per board, gaat dit goed met de losse controllers bij S3Gripper?
        # nee want er zijn twee follow controllers!
        

        #print self.I0, self.I1, self.upper0, self.name0, self.name1

        # set initial values to meaingfull values, esp for fake mode
        self.pos0 = self.lower0         # current position 
        self.vel0 = 0.0                 # current speed
        self.set0 = self.lower0         # setpoint
        self.positions0 =  np.zeros(10) # old positions
        self.velocities0 = np.zeros(10) # old speeds
        self.max_speed0 = 2.1           # max speed. voorlopig vast en alleen in fake mode

        self.pos1 = self.lower1         # current position
        self.vel1 = 0.0                 # current speed
        self.set1 = self.lower1         # setpoint
        self.positions1 =  np.zeros(10) # old positions
        self.velocities1 = np.zeros(10) # old speeds
        self.max_speed1 = 2.1           # max speed. voorlopig vast en alleen in fake mode

        self.last = rospy.Time.now()

        # ROS interfaces
        rospy.Subscriber(self.name0+'/command', Float64, self.command0Cb)
        rospy.Subscriber(self.name1+'/command', Float64, self.command1Cb)
        #rospy.Service(n+'/relax', Relax, self.relaxCb)

    def getPosition(self):
        return [ (self.name0, self.pos0), (self.name1, self.pos1) ]


    def setPosition0(self, pos):
        self.set0 = pos
        if pos > self.upper0:
            self.set0 = self.upper0
        elif pos < self.lower0:
            self.set0 = self.lower0

    def setPosition1(self, pos):
        self.set1 = pos
        if pos > self.upper1:
            self.set1 = self.upper1
        elif pos < self.lower1:
            self.set1 = self.lower1

    def command0Cb(self, req):
        # set self.set0
        # 
        if self.controller0 and self.controller0.active():
            # Under and action control, do not interfere                                                                     
            return
        elif self.set0 != req.data:     #or not self.active:
            setPosition0(self, req.data)


    def command1Cb(self, req):
        # set self.set1
        if self.controller1 and self.controller1.active():
            # Under and action control, do not interfere                                                                     
            return
        elif self.set1 != req.data:     #or not self.active:
            setPosition1(self, req.data)


    def step(self, enc0, enc1):

        if not self.fake:
            # calculate new pos (radians) and speed (radian/sec) from encoder
            # and store in positions and velocities


            # pid code
            # en dat geeft de nieuwe ui

            return pos0, vel0, u0, pos1, vel1, u1

        else:
            # fake stuff
            # bereken de nieuwe position en snelheid
            # gebruik een max speed, voorlopig alleen in fake mode
            t = rospy.Time.now()

            # channel 0
            last_pos0 = self.pos0
            # apply limits to position
            cmd = self.set0 - self.pos0
            if cmd > self.max_speed0/self.rate:
                cmd = self.max_speed0/self.rate
            elif cmd < -self.max_speed0/self.rate:
                cmd = -self.max_speed0/self.rate
            # new position
            self.pos0 = self.pos0 + cmd
            # new speed
            self.vel0 = (self.pos0 - last_pos0)/((t - self.last).to_nsec()/1000000000.0)

            # channel 1
            last_pos1 = self.pos1
            # apply limits to position
            cmd = self.set1 - self.pos1
            if cmd > self.max_speed1/self.rate:
                cmd = self.max_speed1/self.rate
            elif cmd < -self.max_speed1/self.rate:
                cmd = -self.max_speed1/self.rate
            # new position
            self.pos1 = self.pos1 + cmd
            # new speed
            self.vel1 = (self.pos1 - last_pos1)/((t - self.last).to_nsec()/1000000000.0)

            self.last = t

            return self.pos0, self.vel0, 0, self.pos1, self.vel1, 0



class S3Grippercontroller(Board):
    """
       controller for S3 and gripper joints
    """
    def __init__(self,device, DevNr):
        self.fake = device.fake
        self.rate = device.rate

        # parameters voor dit board ophalen
        # these are all constants
        joints = device.joints
        for x in joints:
            if joints[x].get('devnr') == DevNr:
                channr =  joints[x].get('channel')
                if channr == 0:
                    self.name0 = x
                    self.P0 = joints[x].get('P')
                    self.I0 = joints[x].get('I')
                    self.D0 = joints[x].get('D')
                    self.lower0 = joints[x].get('lower')
                    self.upper0 = joints[x].get('upper')
                    self.controller0 = joints[x].get('controller')
                else:
                    self.name1 = x
                    self.P1 = joints[x].get('P')
                    self.I1 = joints[x].get('I')
                    self.D1 = joints[x].get('D')
                    self.lower1 = joints[x].get('lower')
                    self.upper1 = joints[x].get('upper')
                    self.controller1 = joints[x].get('controller')

        #print self.I0, self.I1, self.upper0, self.name0, self.name1

        self.pos0 = self.lower0         # current position
        self.vel0 = 0.0                 # current speed
        self.set0 = self.lower0         # setpoint
        self.positions0 =  np.zeros(10) # old positions
        self.velocities0 = np.zeros(10) # old speeds
        self.max_speed0 = 2.1           # max speed. voorlopig vast en alleen in fake mode

        self.pos1 = self.lower1         # current position
        self.vel1 = 0.0                 # current speed
        self.set1 = self.lower1         # setpoint
        self.positions1 =  np.zeros(10) # old positions
        self.velocities1 = np.zeros(10) # old speeds
        self.max_speed1 = 2.1           # max speed. voorlopig vast en alleen in fake mode

        self.last = rospy.Time.now()

        # ROS interfaces
        rospy.Subscriber(self.name0+'/command', Float64, self.command0Cb)
        rospy.Subscriber(self.name1+'/command', Float64, self.command1Cb)
        #rospy.Service(n+'/relax', Relax, self.relaxCb)


    def getPosition(self):
        return [ (self.name0, self.pos0), (self.name1, self.pos1) ]

    def setPosition0(self, pos):
        self.set0 = pos
        if pos > self.upper0:
            self.set0 = self.upper0
        elif pos < self.lower0:
            self.set0 = self.lower0

    def setPosition1(self, pos):
        self.set1 = pos
        if pos > self.upper1:
            self.set1 = self.upper1
        elif pos < self.lower1:
            self.set1 = self.lower1

    def command0Cb(self, req):
        # set self.set0
        # 
        if self.controller0 and self.controller0.active():
            # Under and action control, do not interfere                                                                     
            return
        elif self.set0 != req.data:     #or not self.active:
            setPosition0(self, req.data)


    def command1Cb(self, req):
        # set self.set1
        if self.controller1 and self.controller1.active():
            # Under and action control, do not interfere                                                                     
            return
        elif self.set1 != req.data:     #or not self.active:
            setPosition1(self, req.data)


    def step(self, enc0, enc1):

        if not self.fake:
            # calculate new pos (radians) and speed (radian/sec) from encoder
            # and store in positions and velocities


            # pid code
            # en dat geeft de nieuwe ui

            return pos0, vel0, u0, pos1, vel1, u1

        else:
            # fake stuff
            # bereken de nieuwe position en snelheid
            # gebruik een max speed, voorlopig alleen in fake mode
            t = rospy.Time.now()

            # channel 0
            last_pos0 = self.pos0
            # apply limits to position
            cmd = self.set0 - self.pos0
            if cmd > self.max_speed0/self.rate:
                cmd = self.max_speed0/self.rate
            elif cmd < -self.max_speed0/self.rate:
                cmd = -self.max_speed0/self.rate
            # new position
            self.pos0 = self.pos0 + cmd
            # new speed
            self.vel0 = (self.pos0 - last_pos0)/((t - self.last).to_nsec()/1000000000.0)

            # channel 1
            last_pos1 = self.pos1
            # apply limits to position
            cmd = self.set1 - self.pos1
            if cmd > self.max_speed1/self.rate:
                cmd = self.max_speed1/self.rate
            elif cmd < -self.max_speed1/self.rate:
                cmd = -self.max_speed1/self.rate
            # new position
            self.pos1 = self.pos1 + cmd
            # new speed
            self.vel1 = (self.pos1 - last_pos1)/((t - self.last).to_nsec()/1000000000.0)

            self.last = t

            return self.pos0, self.vel0, 0, self.pos1, self.vel1, 0


class Elbowcontroller(Board):
    """
       controller for Elbow
           for differential drive
    """
    def __init__(self,device, DevNr):
        self.fake = device.fake
        self.rate = device.rate

        # parameters voor dit board ophalen
        # these are all constants
        joints = device.joints
        for x in joints:
            if joints[x].get('devnr') == DevNr:
                channr =  joints[x].get('channel')
                if channr == 0:
                    self.name0 = x
                    self.P0 = joints[x].get('P')
                    self.I0 = joints[x].get('I')
                    self.D0 = joints[x].get('D')
                    self.lower0 = joints[x].get('lower')
                    self.upper0 = joints[x].get('upper')
                    self.controller0 = joints[x].get('controller')
                else:
                    self.name1 = x
                    self.P1 = joints[x].get('P')
                    self.I1 = joints[x].get('I')
                    self.D1 = joints[x].get('D')
                    self.lower1 = joints[x].get('lower')
                    self.upper1 = joints[x].get('upper')
                    self.controller1 = joints[x].get('controller')

        #print self.I0, self.I1, self.upper0, self.name0, self.name1

        self.pos0 = self.lower0         # current position
        self.vel0 = 0.0                 # current speed
        self.set0 = self.lower0         # setpoint
        self.positions0 =  np.zeros(10) # old positions
        self.velocities0 = np.zeros(10) # old speeds
        self.max_speed0 = 2.1           # max speed. voorlopig vast en alleen in fake mode

        self.pos1 = self.lower1         # current position
        self.vel1 = 0.0                 # current speed
        self.set1 = self.lower1         # setpoint
        self.positions1 =  np.zeros(10) # old positions
        self.velocities1 = np.zeros(10) # old speeds
        self.max_speed1 = 2.1           # max speed. voorlopig vast en alleen in fake mode

        self.last = rospy.Time.now()

        # ROS interfaces
        rospy.Subscriber(self.name0+'/command', Float64, self.command0Cb)
        rospy.Subscriber(self.name1+'/command', Float64, self.command1Cb)
        #rospy.Service(n+'/relax', Relax, self.relaxCb)

    def getPosition(self):
        return [ (self.name0, self.pos0), (self.name1, self.pos1) ]

    def setPosition0(self, pos):
        self.set0 = pos
        if pos > self.upper0:
            self.set0 = self.upper0
        elif pos < self.lower0:
            self.set0 = self.lower0

    def setPosition1(self, pos):
        self.set1 = pos
        if pos > self.upper1:
            self.set1 = self.upper1
        elif pos < self.lower1:
            self.set1 = self.lower1

    def command0Cb(self, req):
        # set self.set0
        # 
        if self.controller0 and self.controller0.active():
            # Under and action control, do not interfere                                                                     
            return
        elif self.set0 != req.data:     #or not self.active:
            setPosition0(self, req.data)


    def command1Cb(self, req):
        # set self.set1
        if self.controller1 and self.controller1.active():
            # Under and action control, do not interfere                                                                     
            return
        elif self.set1 != req.data:     #or not self.active:
            setPosition1(self, req.data)


    def step(self, enc0, enc1):

        if not self.fake:
            # calculate new pos (radians) and speed (radian/sec) from encoder
            # and store in positions and velocities


            # pid code
            # en dat geeft de nieuwe ui

            return pos0, vel0, u0, pos1, vel1, u1

        else:
            # fake stuff
            # bereken de nieuwe position en snelheid
            # gebruik een max speed, voorlopig alleen in fake mode
            t = rospy.Time.now()

            # channel 0
            last_pos0 = self.pos0
            # apply limits to position
            cmd = self.set0 - self.pos0
            if cmd > self.max_speed0/self.rate:
                cmd = self.max_speed0/self.rate
            elif cmd < -self.max_speed0/self.rate:
                cmd = -self.max_speed0/self.rate
            # new position
            self.pos0 = self.pos0 + cmd
            # new speed
            self.vel0 = (self.pos0 - last_pos0)/((t - self.last).to_nsec()/1000000000.0)

            # channel 1
            last_pos1 = self.pos1
            # apply limits to position
            cmd = self.set1 - self.pos1
            if cmd > self.max_speed1/self.rate:
                cmd = self.max_speed1/self.rate
            elif cmd < -self.max_speed1/self.rate:
                cmd = -self.max_speed1/self.rate
            # new position
            self.pos1 = self.pos1 + cmd
            # new speed
            self.vel1 = (self.pos1 - last_pos1)/((t - self.last).to_nsec()/1000000000.0)

            self.last = t

            return self.pos0, self.vel0, 0, self.pos1, self.vel1, 0



class Wristcontroller(Board):
    """
       controller for wrist joints
           for differential drive
    """
    def __init__(self,device, DevNr):
        self.fake = device.fake
        self.rate = device.rate

        # parameters voor dit board ophalen
        # these are all constants
        joints = device.joints
        for x in joints:
            if joints[x].get('devnr') == DevNr:
                channr =  joints[x].get('channel')
                if channr == 0:
                    self.name0 = x
                    self.P0 = joints[x].get('P')
                    self.I0 = joints[x].get('I')
                    self.D0 = joints[x].get('D')
                    self.lower0 = joints[x].get('lower')
                    self.upper0 = joints[x].get('upper')
                    self.controller0 = joints[x].get('controller')
                else:
                    self.name1 = x
                    self.P1 = joints[x].get('P')
                    self.I1 = joints[x].get('I')
                    self.D1 = joints[x].get('D')
                    self.lower1 = joints[x].get('lower')
                    self.upper1 = joints[x].get('upper')
                    self.controller1 = joints[x].get('controller')

        #print self.I0, self.I1, self.upper0, self.name0, self.name1

        self.pos0 = self.lower0         # current position
        self.vel0 = 0.0                 # current speed
        self.set0 = self.lower0         # setpoint
        self.positions0 =  np.zeros(10) # old positions
        self.velocities0 = np.zeros(10) # old speeds
        self.max_speed0 = 2.1           # max speed. voorlopig vast en alleen in fake mode

        self.pos1 = self.lower1         # current position
        self.vel1 = 0.0                 # current speed
        self.set1 = self.lower1         # setpoint
        self.positions1 =  np.zeros(10) # old positions
        self.velocities1 = np.zeros(10) # old speeds
        self.max_speed1 = 2.1           # max speed. voorlopig vast en alleen in fake mode

        self.last = rospy.Time.now()

        # ROS interfaces
        rospy.Subscriber(self.name0+'/command', Float64, self.command0Cb)
        rospy.Subscriber(self.name1+'/command', Float64, self.command1Cb)
        #rospy.Service(n+'/relax', Relax, self.relaxCb)


    def getPosition(self):
        return [ (self.name0, self.pos0), (self.name1, self.pos1) ]

    def setPosition0(self, pos):
        self.set0 = pos
        if pos > self.upper0:
            self.set0 = self.upper0
        elif pos < self.lower0:
            self.set0 = self.lower0

    def setPosition1(self, pos):
        self.set1 = pos
        if pos > self.upper1:
            self.set1 = self.upper1
        elif pos < self.lower1:
            self.set1 = self.lower1

    def command0Cb(self, req):
        # set self.set0
        # 
        if self.controller0 and self.controller0.active():
            # Under and action control, do not interfere                                                                     
            return
        elif self.set0 != req.data:     #or not self.active:
            setPosition0(self, req.data)


    def command1Cb(self, req):
        # set self.set1
        if self.controller1 and self.controller1.active():
            # Under and action control, do not interfere                                                                     
            return
        elif self.set1 != req.data:     #or not self.active:
            setPosition1(self, req.data)


    def step(self, enc0, enc1):

        if not self.fake:
            # calculate new pos (radians) and speed (radian/sec) from encoder
            # and store in positions and velocities


            # pid code
            # en dat geeft de nieuwe ui

            return pos0, vel0, u0, pos1, vel1, u1

        else:
            # fake stuff
            # bereken de nieuwe position en snelheid
            # gebruik een max speed, voorlopig alleen in fake mode
            t = rospy.Time.now()

            # channel 0
            last_pos0 = self.pos0
            # apply limits to position
            cmd = self.set0 - self.pos0
            if cmd > self.max_speed0/self.rate:
                cmd = self.max_speed0/self.rate
            elif cmd < -self.max_speed0/self.rate:
                cmd = -self.max_speed0/self.rate
            # new position
            self.pos0 = self.pos0 + cmd
            # new speed
            self.vel0 = (self.pos0 - last_pos0)/((t - self.last).to_nsec()/1000000000.0)

            # channel 1
            last_pos1 = self.pos1
            # apply limits to position
            cmd = self.set1 - self.pos1
            if cmd > self.max_speed1/self.rate:
                cmd = self.max_speed1/self.rate
            elif cmd < -self.max_speed1/self.rate:
                cmd = -self.max_speed1/self.rate
            # new position
            self.pos1 = self.pos1 + cmd
            # new speed
            self.vel1 = (self.pos1 - last_pos1)/((t - self.last).to_nsec()/1000000000.0)

            self.last = t

            return self.pos0, self.vel0, 0, self.pos1, self.vel1, 0












