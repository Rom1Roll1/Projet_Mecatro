#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 12 14:29:53 2018

@author: baptistejouk
"""
import seabreeze.spectrometers as sb
class Spectro(object):
    def _init_(self):
        self = sb.Spectrometer.from_serial_number()
        self.integration_time_micros(10000)
        
        
    def getSpectra(self):
        return [self.wavelengths(),self.intensities()]
    def close(self):
        self.close()