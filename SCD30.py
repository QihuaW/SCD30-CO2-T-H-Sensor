# -*- coding: utf-8 -*-
"""
Created on Fri Sep 24 14:21:23 2021

@author: Qihua Wu

Back end py for communicating with the SCD30 CO2, T/H sensor through ncd i2c-usb bridge
"""
#Imports
import time
import numpy as np
import ncd_usb_i2c as ncd
import struct


class cmds:
    """
    Commands to send to SCD30 
    Commands = [ [register_1, register_2], read_length, wati_time (ms)]
    """
    con_measure = [[0x00, 0x10], 0, 10]
    read_measure = [[0x03, 0x00], 18, 50]
    set_interval = [[0x46, 0x00], 0, 10]
    ready_status = [[0x02, 0x02], 3, 20]
    stop_measure = [[0x01, 0x04], 0, 10]
    temp_offset = [[0x54, 0x03], 0, 10]
    auto_calibration = [[0x53, 0x06], 0, 10]
    forced_recali_value = [[0x52, 0x04], 0, 10]
    altitude_compens = [[0x51, 0x02], 0, 10]


class Crc8:
    # For calculating CRC8 checksum for SCD30
    def __init__(s):
        s.crc = 255

    def hash(s, int_list):
        for i in int_list:
            s.addVal(i)
        return s.crc

    def addVal(s, n):
        crc = s.crc
        for bit in range(0, 8):
            if ( n ^ crc ) & 0x80:
                crc = ( crc << 1 ) ^ 0x31 # Polynomial is 0x31 for SCD30
            else:
                crc = ( crc << 1 )
            n = n << 1
        s.crc = crc & 0xFF    # Initial Value is 0xFF for SCD30
        return s.crc


class _convert:
    # For data pack and unpack
    def convert(self,list):
        # Take a list of 32 bit data (4 int or float) and return single float 
        data = bytearray(list)  
        return struct.unpack('>f', data)
    
    def pack_msg(self, n):
        # Pack message/commands and checksum into a list for ncd bridge
        n_hex = [(n >> 8), (n & 0xFF)]      # Pack the message into 16 bit data list
        checksum = [(Crc8().hash(n_hex))]   # Create CRC8 checksum and convert into a list
        return n_hex + checksum

class scd30:
    def __init__(self, port):

        # Use ncd connector port
        self.handle = ncd.usb_i2c(port)     # port is the COM port of ncd bridge
        self.address = 0x61     # SCD30 i2c address is 0x61


    def write(self, cmd, msg = None):
        if msg == None:
            self.handle.write(self.address, cmd[0])
        else:
            self.handle.write(self.address, cmd[0], msg)
        time.sleep(cmd[2] / 1000)

    def read(self, cmd):
        response = self.handle.read(self.address, cmd[0], cmd[1])
        time.sleep(cmd[2] / 1000)
        return response

    def con_measure(self, msg = 0):
        """
        Take 0 or 1 integer as pressure compensation;
        Available range: 0 & 700 to 1200 (pressure in mBar)
        No input: deactivates pressure compensation;
        
        Send the command to trigger continous measurement (2s interval)
        """
        if msg != 0:
            msg = _convert().pack_msg(msg)
        self.init = self.write(cmds.con_measure, msg)
    
    def stop_measure(self):
        """
        Stop the continous measurement 
        """
        self.init = self.write(cmds.stop_measure, 1)

    def read_measure(self):
        """
        Read out when new data is available.
        
        Return 18 Bytes of message (32 bit/big-endian):
        CO2 concentration (in ppm): Byte 1,2 & 4,5
        Temperature (in C): Byte 7,8 & 10,11
        Humidity (%): Byte 13,14 & 17,18
        """    
        data = self.read(cmds.read_measure)
        CO2_data = _convert().convert(data[2:4] + data[5:7])[0]
        temp_data = _convert().convert(data[8:10] + data[11:13])[0]
        hum_data = _convert().convert(data[14:16] + data[17:19])[0]
        return [round(CO2_data,2), round(temp_data,2),  round(hum_data,2)]
    
    def set_interval(self, interval):
        """
        Send 1 integer as measurement interval: range 2 to 1800 in sec
        
        """
        self.write(cmds.set_interval, _convert().pack_msg(interval))

    def read_status(self):
        """
        Get data ready status - 1: ready or 0: not ready
        """
        data = self.read(cmds.ready_status)
        status = int.from_bytes(data[2:4], 'big')
        return status

    def set_temp_offset(self, temp):
        """
        Set temperature offset: 1 integer of temp offset in C 
        """
        self.write(cmds.temp_offset, _convert().pack_msg(temp*100))
        
    def set_auto_calibration(self, msg):
        """
        Set auto calibration mode (default is 0)
        1: Activate auto calibration
        0: Deactivate auto calibraiton
        """
        if msg == 0 or msg == 1:
            self.write(cmds.auto_calibration, _convert().pack_msg(msg))
        else: print('Auto Calibration Setting can only be 1 or 0 !')
    
    def set_force_recali(self, value):
        """
        Set forced recalibration reference
        Take 1 integer of CO2 reference value: range from 400 to 2000 (ppm)
        """
        if value >= 400 and value <= 2000:
            self.write(cmds.forced_recali_value, _convert().pack_msg(value))
        else: print('Reference value has to be within 400 to 2000ppm !')
            
    def altitude_compens(self, value):
        """
        Set altitude compensation: input 1 integer (>0)
        """
        if value >= 0 and value <= 5000:
            self.write(cmds.altitude_compens, _convert().pack_msg(value))
        else: print('Altitude value has to be within 0 to 5000m !')


if __name__ == "__main__":
    sensor = scd30('COM3')
    time.sleep(0.05)
    sensor.set_interval(2)  
    #sensor.set_temp_offset(0) 
    #sensor.set_auto_calibration(0)
    time.sleep(0.05)
    sensor.con_measure()
    print('Start Continuous Measurement')
    t = 0
    while t < 10:
        t = t + 1
        status = sensor.read_status()
        if status == 1:
            results = sensor.read_measure()
            print(t, ' ', results)
        else:
            pass
        time.sleep(2)

            

          
        