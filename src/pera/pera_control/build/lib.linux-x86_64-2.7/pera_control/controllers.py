#!/usr/bin/env python

#   base class for controllers for the PERA
#    derived from arbotix version
class Controller:

    def __init__(self, device, name):
        self.name = name
        self.device = device
        self.pause = False

        # output for joint states publisher
        self.joint_names = list()
        self.joint_positions = list()
        self.joint_velocities = list()

    ## @brief Start the controller, do any hardware setup needed.
    def startup(self):
        pass

    ## @brief Do any read/writes to device.
    def update(self):
        pass

    ## @brief Stop the controller, do any hardware shutdown needed.
    def shutdown(self):
        pass

    ## @brief Is the controller actively sending commands to joints?
    def active(self):
        return False
        
    ## @brief Get a diagnostics message for this joint.
    ##
    ## @return Diagnostics message. 
    def getDiagnostics(self):
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        return msg

