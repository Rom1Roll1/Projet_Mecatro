#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 13 11:14:09 2018

@author: baptistejouk
"""
import os, dynamixel, time, random
import sys
from serial import Serial

class Moteurs(object):
    def _init_(self):
        # The number of Dynamixels on our bus.
        nServos = 3
        # Set your serial port accordingly.
        if os.name == "posix":
            portName = "/dev/ttyUSB0"
        else:
            portName = "COM5"
    
        # Default baud rate of the USB2Dynamixel device.
        baudRate = 1000000
        
        self.serial = dynamixel.SerialStream( port=portName, baudrate=baudRate, timeout=1)
        self.net = dynamixel.DynamixelNetwork( self.serial )
        
        sys.stdout.flush()
        
        self.net.scan( 1, nServos )
        myActuators = list()

        for dyn in self.net.get_dynamixels():
            myActuators.append(dyn)
        self.Moteurs=myActuators
        return self
        
    
    def close(self):
        self.serial.close()