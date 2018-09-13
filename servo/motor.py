# -*- coding: utf-8 -*-
"""
Created on Thu Sep 13 11:14:09 2018

@author: baptistejouk
"""
import os, time,dynamixel, random
import sys
from serial import Serial

class Moteurs(object):
    
    def __init__(self):
        # The number of Dynamixels on our bus.
        nServos = 10
        # Set your serial port accordingly.
        if os.name == "posix":
            portName = "/dev/tty.usbserial-A94N75T1"
        else:
            portName = "COM5"
        # Default baud rate of the USB2Dynamixel device.
        baudRate = 1000000
        
        self.serial = dynamixel.SerialStream( port=portName, baudrate=baudRate, timeout=1)
        self.net = dynamixel.DynamixelNetwork( self.serial )
        
        sys.stdout.flush()
        
        self.net.scan( 0, nServos )
        myActuators = list()
        
        for dyn in self.net.get_dynamixels():
            myActuators.append(dyn)
        self.Moteurs=myActuators
    
    def conversion_angle_interval( theta ):
        """ Take an angle and convert it into a cervomotor interval """
        minimum_angle = 300 / 1024 #equivalent in ° of a motor interval
        targeted_interval = round (theta / minimum_angle) #interger equivalent of the targeted angle in motor interval
        return targeted_interval
        
        
        
    def move_motor(self, ID, theta_f, V = 255):
        """ Take a motor ID, an ordered angle and the motor speed and make it move """
        self.Moteurs.goal_position[ID] = conversion_angle_interval( theta_f )
        net.synchronize()
        self.Moteurs.read_all() #read all the properties of the cervomotor
        time.sleep(2) #Need time to move
        residual_error = self.Moteurs.current_position - conversion_angle_interval (theta_f)
    
    def close(self):
        self.serial.close()
    
    