#!/usr/bin/env python


class Pid1 (Controller):

    def __init__(self, dev):
        # parameters 

nee, deze parameters in de echte controller doen.
dus in de klasse van Pid1 en zo.

        # alleen de joints die bij dit device nummer horen
        # PID parameters and stuff
        # channel X
        self.XP =  int(rospy.get_param(n+"X/P", "0"))
        self.XI =  int(rospy.get_param(n+"X/I", "0"))
        self.XD =  int(rospy.get_param(n+"X/D", "0"))
        control =  rospy.get_param(n+"X/type", "dummy")

        self.controlX =
        self.controlY =  rospy.get_param(n+"X/type", DummyController)
        # nee

    # setpoint and process variable
    def step(self, setpoint, pv):
        print 'een stap'
        return iets
