# -*- coding: utf-8 -*-
"""
Created on Thu Sep 13 11:14:09 2018

@author: baptistejouk
"""
import os, time,dynamixel, random
import sys
from serial import Serial

class Moteurs(object):
    
    def __init__(self,parameters= 0):
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
        self.listeMoteurs=myActuators
        '''Define parameters for angles: list: 1 row=> 1 motor, first column: 0 angle, second column positive/negative rotation'''
        if parameters!=0:
            self.parameters=parameters
        else:
            for act in range(0,len(self.listeMoteurs)):
                self.parameters.append([0,1])
            
    
    def conversion_angle_interval(self, theta ,ID):
        """ Take an angle and convert it into a cervomotor interval """
        targeted_interval = self.parameters[ID][1]*round(theta / (300./1023))-self.parameters[ID][0] #interger equivalent of the targeted angle in motor interval
        return targeted_interval
    
    def conversion_interval_angle(self, interval ,ID):
        """ Take an angle and convert it into a cervomotor interval """
        angle = self.parameters[ID][1]*((interval-self.parameters[ID][0]) * 300 / 1023.) #integer equivalent of the targeted angle in motor interval
        return angle
        
        
    def move_motor(self, ID, theta_f, V = 255):
        """ Take a motor ID, an ordered angle and the motor speed and make it move
        Return the residual error on the move"""
        self.listeMoteurs[ID].goal_position = int(self.conversion_angle_interval( theta_f ,ID))
        self.net.synchronize()
        self.listeMoteurs[ID].read_all() #read all the properties of the cervomotor
        time.sleep(0.2)
        self.listeMoteurs[ID].read_all() #read all the properties of the cervomotor
        while self.listeMoteurs[ID].current_speed!=0:
            time.sleep(0.1)#Need time to move
        self.listeMoteurs[ID].read_all() #read all the properties of the cervomotor
        residual_error = self.listeMoteurs[ID].current_position - self.conversion_angle_interval (theta_f ,ID)
        return self.conversion_interval_angle (residual_error)
    
    def close(self):
        self.serial.close()
    
    