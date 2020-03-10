# -*- coding: utf-8 -*-
#! /usr/bin/env python

"""Module to interface with CH Robotics / Redshift Labs UM7 IMU
"""

# Copyright (c) 2016 by Till Busch <till@bux.at>
# based on
# Daniel Kurek, d'Arbeloff Lab, MIT, January, 2016
# Published under MIT LICENSE
# Module that holds UM7 class
# Creates serial objects, contains functions to parse serial data

# TODO: Settings, Calibration, Matrices

import serial
import time
import binascii
import struct
import sys

try:
    monotonic = time.monotonic
except AttributeError:
    from monotonic import monotonic

GET_FW_REVISION       = 0xAA # (170)
FLASH_COMMIT          = 0xAB # (171)
RESET_TO_FACTORY      = 0xAC # (172)
ZERO_GYROS            = 0xAD # (173)
SET_HOME_POSITION     = 0xAE # (174)
SET_MAG_REFERENCE     = 0xB0 # (176)
RESET_EKF             = 0xB3 # (179)

DREG_HEALTH           = 0x55
DREG_GYRO_RAW_XY      = 0x56
DREG_GYRO_PROC_X      = 0x61
DREG_ACCEL_PROC_X     = 0x65
DREG_QUAT_AB          = 0x6D
DREG_EULER_PHI_THETA  = 0x70
DREG_GYRO_BIAS_X      = 0x89

CREG_COM_SETTINGS     = 0x00
CREG_COM_RATES1       = 0x01
CREG_COM_RATES2       = 0x02
CREG_COM_RATES3       = 0x03
CREG_COM_RATES4       = 0x04
CREG_COM_RATES5       = 0x05
CREG_COM_RATES6       = 0x06
CREG_COM_RATES7       = 0x07
CREG_GYRO_TRIM_X      = 0x0C
CREG_MAG_CAL1_1       = 0x0F
CREG_MAG_BIAS_X       = 0x18
CREG_ACCEL_CAL1_1     = 0x1B
CREG_ACCEL_BIAS_X     = 0x24

CREG_MISC_SETTINGS    = 0x08
CREG_MISC_SETTINGS_MAG= 0x01
CREG_MISC_SETTINGS_Q  = 0x02
CREG_MISC_SETTINGS_ZG = 0x04
CREG_MISC_SETTINGS_PPS= 0x100


REG_HIDDEN            = 0xF000
H_CREG_GYRO_VARIANCE  = REG_HIDDEN | 0x00
H_CREG_ACCEL_VARIANCE = REG_HIDDEN | 0x01
H_CREG_MAG_VARIANCE   = REG_HIDDEN | 0x02
H_CREG_ACCEL_TAU      = REG_HIDDEN | 0x11
H_CREG_GYRO_TAU       = REG_HIDDEN | 0x12
H_CREG_MAG_TAU        = REG_HIDDEN | 0x13
H_CREG_GYRO_ALIGN1_1  = REG_HIDDEN | 0x31
H_CREG_ACCEL_ALIGN1_1 = REG_HIDDEN | 0x52
H_CREG_MAG_ALIGN1_1   = REG_HIDDEN | 0x73
H_CREG_MAG_REF        = REG_HIDDEN | 0x7C

HEALTH_GPS   = 0x1
HEALTH_MAG   = 0x2
HEALTH_GYRO  = 0x4
HEALTH_ACCEL = 0x8
HEALTH_ACC_N = 0x10
HEALTH_MG_N  = 0x20
HEALTH_RES6  = 0x40
HEALTH_RES7  = 0x80
HEALTH_OVF   = 0x100

class UM7Packet(object):
    def __init__(self, foundpacket=False, hasdata=False, startaddress=0x0, data=None, commandfailed=True, timeout=False):
        self.foundpacket = bool(foundpacket)
        self.hasdata = bool(hasdata)
        self.startaddress = startaddress
        self.data = data
        self.commandfailed = bool(commandfailed)
        self.timeout = bool(timeout)

    def __str__(self):
        return '{}(start=0x{:x}, found={}, failed={}, timeout={}, hasdata={})'.format(self.__class__.__name__, self.startaddress, self.foundpacket, self.commandfailed, self.timeout, self.hasdata)

class UM7(object):
    """ Class that handles UM7 interfacing. Creates serial object for communication, contains functions to request specific
        data samples, catch any incoming data, check input buffer, and set various data broadcast rates. Currently only
        handles processed accel, gyro, and euler angle data.  Data is timed by OS.
    """
    baud_rates = { 9600: 0, 14400: 1, 19200: 2, 38400: 3, 57600: 4, 115200: 5, 128000: 6, 153600: 7, 230400: 8, 256000: 9, 460800: 10, 921600: 11 }

    def __init__(self, name, port, statevars, baud=115200):
        """Create new UM7 serial object.
        Default Baud Rate = 115200
        Initializes port, name, OS timer, and sensor state (dict)
        :param port: Virtual COM port to which the IMU is connected (str)
               name: name of object (str)
        """
        super(UM7, self).__init__()
        statevars[:] = [i for i in statevars]
        self.name = name
        self.t0 = monotonic()
        self.state = {}
        self.statevars = statevars
        self.serial = None
        for i in statevars:
            self.state.update({i: 0})
        try:
            self.serial = serial.Serial(port, baudrate=baud, bytesize=8, parity='N', stopbits=1, timeout=0.1)
        except OSError:
            print('Could not connect to UM7 %s.' % self.name)

    def __del__(self):
        if self.serial:
            self.serial.close()

    def __name__(self):
        return self.name

    def catchsample(self):
        """Function that catches and parses incoming data, and then updates the sensor's state to include new data. Old
        data in state is overwritten.

        :return: Newly obtained data, and updates internal sensor state
        """
        packet = self.readpacket()
        if not packet.foundpacket:
            return False
        sample = self.parsedatabatch(packet.data, packet.startaddress)
        if sample:
            self.state.update(sample)
        return sample

    def catchallsamples(self, wanted_state, timeout):
        sample = {}
        t0 = monotonic()
        all_found = False
        while monotonic() - t0 < timeout:
            packet = self.readpacket()
            if packet.foundpacket:
                newsample = self.parsedatabatch(packet.data, packet.startaddress)
                if newsample:
                    sample.update(newsample)
            if all (k in sample for k in wanted_state): # all vars found
                all_found = True
                break
        self.state.update(sample)
        return all_found

    def readpacket(self, timeout=0.1):
        """Scans for and partially parses new data packets. Binary data can then be sent to data parser

        :return: Parsed packet info
        """
        foundpacket = 0
        t0 = monotonic()
        while monotonic() - t0 < timeout:  # While elapsed time is less than timeout
            if self.serial.inWaiting() >= 3:
                byte = self.serial.read(size=1)
                if byte == b's':
                    byte2 = self.serial.read(size=1)
                    if byte2 == b'n':
                        byte3 = self.serial.read(size=1)
                        if byte3 == b'p':
                            foundpacket = 1
                            break
                else:
                    #print(byte)
                    pass
            else:
                time.sleep(0.01)
        if foundpacket == 0:
            hasdata = 0
            commandfailed = 0
            startaddress = 0
            data = 0
            timeout = 1
        else:
            timeout = 0
            try:
                pt = bytearray(self.serial.read(size=1))[0]
                #print(bin(pt))
                hasdata = pt & 0b10000000
                isbatch = pt & 0b01000000
                numdatabytes = ((pt & 0b00111100) >> 2) * 4
                #print('numdatabytes={}'.format(numdatabytes))
                commandfailed = pt & 0b00000001
                hidden = pt & 0b00000010
                if not isbatch:
                    numdatabytes = 4

                startaddress = bytearray(self.serial.read(size=1))[0]
                #print('start={}'.format(startaddress))
                while self.serial.inWaiting() < numdatabytes:
                    pass
                if hasdata:
                    data = bytearray(self.serial.read(size=numdatabytes))
                else:
                    data = False
                cs = bytearray(self.serial.read(size=2))
                cs = struct.unpack('!h', cs)[0]
                ocs = 0
                ocs += ord('s')
                ocs += ord('n')
                ocs += ord('p')
                ocs += pt
                ocs += startaddress
                if data:
                    ocs += sum(data)
                if hidden: startaddress |= REG_HIDDEN
                if ocs != cs:
                    print('bad checksum: {:4x} (should be: {:4x})'.format(cs, ocs))
                    raise ValueError
            except ValueError:
                hasdata = 0
                commandfailed = 0
                startaddress = 0
                data = 0
        return UM7Packet(foundpacket, hasdata, startaddress, data, commandfailed, timeout)

    def readreg(self, start, length=0, timeout=0.1):
        if not self.serial:
            return UM7Packet(startaddress=start, timeout=True)
        hidden = start & REG_HIDDEN
        sa = start & 0xFF
        pt = 0x0
        if length:
            pt = 0b01000000
            pt |= (length << 2)
        if hidden:
            pt |= 0b00000010
        ba = bytearray([ord('s'), ord('n'), ord('p'), pt, sa])
        cs = sum(ba)
        ba += struct.pack('!h', cs)
        self.serial.write(ba)
        t0 = monotonic()
        while monotonic() - t0 < timeout:  # While elapsed time is less than timeout
            packet = self.readpacket()
            if packet.startaddress == start:
                return packet
        return UM7Packet(startaddress=start, timeout=True)

    def writereg(self, start, length=0, data=None, timeout=0.1, no_read=False):
        if not self.serial:
            return UM7Packet(startaddress=start, timeout=True)
        hidden = start & REG_HIDDEN
        sa = start & 0xFF
        pt = 0x0
        if data:
            pt = 0b11000000
            pt |= (length << 2)
        if hidden:
            pt |= 0b00000010
        ba = bytearray([ord('s'), ord('n'), ord('p'), pt, sa])
        if data:
            ba += data
        cs = sum(ba)
        ba += struct.pack('!h', cs)
        self.serial.write(ba)
        if no_read:
            self.serial.flush()
            return UM7Packet(startaddress=start)
        t0 = monotonic()
        while monotonic() - t0 < timeout:  # While elapsed time is less than timeout
            packet = self.readpacket()
            if packet.startaddress == start:
                return packet
        return UM7Packet(startaddress=start, timeout=True)

    def zero_gyros(self):
        """Sends request to zero gyros and waits for confirmation from sensor

        :return: True or False based on success of request
        """
        p = self.writereg(ZERO_GYROS)
        return (not p.commandfailed)

    def reset_ekf(self):
        """Sends request to reset ekf and waits for confirmation from sensor

        :return: True or False based on success of request
        """
        p = self.writereg(RESET_EKF)
        return (not p.commandfailed)

    def reset_to_factory(self):
        p = self.writereg(RESET_TO_FACTORY)
        return (not p.commandfailed)

    def set_mag_reference(self):
        p = self.writereg(SET_MAG_REFERENCE)
        return (not p.commandfailed)

    def set_home_position(self):
        p = self.writereg(SET_HOME_POSITION)
        return (not p.commandfailed)

    def flash_commit(self):
        p = self.writereg(FLASH_COMMIT)
        return (not p.commandfailed)

    def get_fw_revision(self):
        p = self.readreg(GET_FW_REVISION)
        if p.commandfailed:
            return False
        return p.data.decode()

    def set_misc(self, bit, val):
        p = self.readreg(CREG_MISC_SETTINGS)
        if p.commandfailed:
            return False
        cr = struct.unpack('!I', p.data)[0]
        print('{:032b}'.format(cr))
        if(val):
            cr |= bit
        else:
            cr &= ~bit
        print('{:032b}'.format(cr))
        p = self.writereg(start=CREG_MISC_SETTINGS, length=1, data=struct.pack('!I', cr))
        return (not p.commandfailed)

    def set_baud_rate(self, baud):
        new_baud = self.baud_rates[baud] << 28
        p = self.readreg(CREG_COM_SETTINGS)
        if p.commandfailed:
            return False
        cr = struct.unpack('!I', p.data)[0]
        print('{:032b}'.format(cr))
        cr &= 0x0fffffff
        cr |= new_baud
        print('{:032b}'.format(cr))
        p = self.writereg(start=CREG_COM_SETTINGS, length=1, data=struct.pack('!I', new_baud), no_read=True)
        if not p:
            return False
        self.serial.baudrate = baud

    @staticmethod
    def parsedatabatch(data, startaddress):
        address = startaddress
        offset = 0
        output = {}
        while offset < len(data):
            rdata = UM7RegInfo.getdata(address)
            if rdata is not None:
                rformat, rnames, rscale = rdata
                vals = struct.unpack(rformat, data[offset:offset+4])
                if rscale is not None:
                    vals = map(lambda x: x * rscale, vals)
                output.update(zip(rnames, vals))
            address += 1
            offset += 4
        return output


class UM7RegInfo(object):
    """
    Data class for UM7 registers
    """
    _degreescale = 1/91.02222         # scale factor for degrees
    _ratescale = 1/16.0               # scale factor for rate
    _quaternionscale = 1/29789.09091  # scale factor for quaternion element

    # register data for UM7 reported registers between _regdatastart and _regdataend inclusive
    # each register data element contains a unpack patter, list of register data names and an optional scale factor
    _regdatastart = 0x55
    _regdata = [('!i', ('health',), None),
                ('!hh', ('gyro_raw_x', 'gyro_raw_y'), _degreescale),
                ('!h2x', ('gyro_raw_z',), _degreescale),
                ('!f', ('gyro_raw_time',), None),
                ('!hh', ('accel_raw_x', 'accel_raw_y'), None),
                ('!h2x', ('accel_raw_z',), None),
                ('!f', ('accel_raw_time',), None),
                ('!hh', ('mag_raw_x', 'mag_raw_y'), None),
                ('!h2x', ('mag_raw_z',), None),
                ('!f', ('mag_raw_time',), None),
                ('!f', ('temp',), None),
                ('!f', ('temp_time',), None),
                ('!f', ('gyro_proc_x',), None),
                ('!f', ('gyro_proc_y',), None),
                ('!f', ('gyro_proc_z',), None),
                ('!f', ('gyro_proc_time',), None),
                ('!f', ('accel_proc_x',), None),
                ('!f', ('accel_proc_y',), None),
                ('!f', ('accel_proc_z',), None),
                ('!f', ('accel_proc_time',), None),
                ('!f', ('mag_proc_x',), None),
                ('!f', ('mag_proc_y',), None),
                ('!f', ('mag_proc_z',), None),
                ('!f', ('mag_proc_time',), None),
                ('!hh', ('quat_a', 'quat_b'), _quaternionscale),
                ('!hh', ('quat_c', 'quat_d'), _quaternionscale),
                ('!f', ('quat_time',), None),
                ('!hh', ('roll', 'pitch'), _degreescale),
                ('!h2x', ('yaw',), _degreescale),
                ('!hh', ('roll_rate', 'pitch_rate'), _ratescale),
                ('!h2x', ('yaw_rate',), _ratescale),
                ('!f', ('euler_time',), None),
                ('!f', ('position_n',), None),
                ('!f', ('position_e',), None),
                ('!f', ('position_up',), None),
                ('!f', ('position_time',), None),
                ('!f', ('velocity_n',), None),
                ('!f', ('velocity_e',), None),
                ('!f', ('velocity_up',), None),
                ('!f', ('velocity_time',), None),
                ('!f', ('gps_latitude',), None),
                ('!f', ('gps_longitude',), None),
                ('!f', ('gps_altitude',), None),
                ('!f', ('gps_course',), None),
                ('!f', ('gps_speed',), None),
                ('!f', ('gps_time',), None),
                ('!BbBb', ('gps_sat1_id', 'gps+sat1_snr', 'gps_sat2_id', 'gps_sat2_snr'), None),
                ('!BbBb', ('gps_sat3_id', 'gps+sat3_snr', 'gps_sat4_id', 'gps_sat4_snr'), None),
                ('!BbBb', ('gps_sat5_id', 'gps+sat5_snr', 'gps_sat6_id', 'gps_sat6_snr'), None),
                ('!BbBb', ('gps_sat7_id', 'gps+sat7_snr', 'gps_sat8_id', 'gps_sat8_snr'), None),
                ('!BbBb', ('gps_sat9_id', 'gps+sat9_snr', 'gps_sat10_id', 'gps_sat10_snr'), None),
                ('!BbBb', ('gps_sat11_id', 'gps+sat11_snr', 'gps_sat12_id', 'gps_sat12_snr'), None),
                ('!f', ('gyro_bias_x',), None),
                ('!f', ('gyro_bias_y',), None),
                ('!f', ('gyro_bias_z',), None)]
    _regdataend = _regdatastart + len(_regdata)-1

    @classmethod
    def getdata(cls, regno):
        if UM7RegInfo._regdatastart <= regno <= UM7RegInfo._regdataend:
            return UM7RegInfo._regdata[regno-UM7RegInfo._regdatastart]
        else:
            return None



