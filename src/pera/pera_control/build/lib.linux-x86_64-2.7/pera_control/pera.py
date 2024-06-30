#!/usr/bin/env python


import time, sys, thread
from struct import unpack
#import rtmotion_usb_conf


## @brief This class controls an ArbotiX, USBDynamixel, or similar board through a serial connection.
class PERA:

    ## @brief Constructs an PERA instance 
    ##
    ## @param port The name of the serial port to open.
    ## 
    ## @param baud The baud rate to run the port at. 
    ##
    ## @param timeout The timeout to use for the port. When operating over a wireless link, you may need to
    ## increase this.
    def __init__(self, port="/dev/ttyUSB0",baud=115200, timeout = 0.1):
        print "init PERA"

  
