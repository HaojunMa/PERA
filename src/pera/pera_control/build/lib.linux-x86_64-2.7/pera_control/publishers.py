#!/usr/bin/env python

"""
"""

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState

class DiagnosticsPublisher:
    """ Class to handle publications of joint_states message. """

    def __init__(self):
        self.t_delta = rospy.Duration(1.0/rospy.get_param("~diagnostic_rate", 1.0))
        self.t_next = rospy.Time.now() + self.t_delta
        self.pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=5)

    def update(self, joints, controllers):
        """ Publish diagnostics. """    
        now = rospy.Time.now()
        if now > self.t_next:
            # create message
            msg = DiagnosticArray()
            msg.header.stamp = now
            for controller in controllers:
                d = controller.getDiagnostics()
                if d:
                    msg.status.append(d)
            for joint in joints:
                d = joint.getDiagnostics()
                if d:
                    msg.status.append(d)
            # publish and update stats
            self.pub.publish(msg)
            self.t_next = now + self.t_delta
        

class JointStatePublisher:
    """ Class to handle publications of joint_states message. """

    def __init__(self):
        # parameters: throttle rate and geometry
        # nog doen, publish rate ontkoppelen van controllerlooprate

        # subscriber
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=5)

    def update(self, names, position, velocity):
        """ publish joint states. """
        #if rospy.Time.now() > self.t_next:   
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = names
        msg.position = position
        msg.velocity = velocity
        self.pub.publish(msg)


