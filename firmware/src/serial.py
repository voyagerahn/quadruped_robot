#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: jaesungAhn
"""

import serial
import time
import numpy as np


class ArduinoSerial:  # class for communication with arduino via serial, starts saying port and speed
    # and in the function create a string with the format for reading.
    # in the first execution you have to open the port
    def __init__(self, port):
        # ls -l /dev | grep ACM to identify serial port of the arduino
        self.arduino = serial.Serial(port, 115200, timeout=1)
        # Note: we cause a manual reset of the plate to read from
        self.arduino.setDTR(False)
        time.sleep(1)
        self.arduino.flushInput()
        self.arduino.setDTR(True)
        time.sleep(2)
        self.lastTime = 0.
        self.previousMillis = 0.
        self.interval = 0.02  # arduino loop running at 20 ms

    def serialSend(self, FR_angles,FL_angles,BR_angles,BL_angles):
        comando = "<{0}#{1}#{2}#{3}#{4}#{5}#{6}#{7}#{8}#{9}#{10}#{11}>"  # Input
        command = comando.format(int(np.rad2deg(FR_angles[0])), int(np.rad2deg(FR_angles[1])), int(np.rad2deg(FR_angles[2])),
                                 int(np.rad2deg(FL_angles[0])), int(np.rad2deg(FL_angles[1])), int(np.rad2deg(FL_angles[2])),
                                 int(np.rad2deg(BR_angles[0])), int(np.rad2deg(BR_angles[1])), int(np.rad2deg(BR_angles[2])),
                                 int(np.rad2deg(BL_angles[0])), int(np.rad2deg(BL_angles[1])), int(np.rad2deg(BL_angles[2])))
        self.arduino.write(bytes(command, encoding='utf8'))  # Send a command to Arduino

        # print(command)
        self.lastTime = time.time()

    def serialRecive(self):
        try:
            startMarker = 60
            endMarker = 62

            getSerialValue = bytes()
            x = "z"  # any value that is not an end- or startMarker
            byteCount = -1  # to allow for the fact that the last increment will be one too many

            # wait for the start character
            while ord(x) != startMarker:
                x = self.arduino.read()

                # save data until the end marker is found
            while ord(x) != endMarker:
                if ord(x) != startMarker:
                    getSerialValue = getSerialValue + x
                    byteCount += 1
                x = self.arduino.read()

            loopTime, Xacc, Yacc, roll, pitch = np.fromstring(getSerialValue.decode('ascii', errors='replace'),
                                                                 sep='#')
        #             print(loopTime , roll , pitch)

        except ValueError:
            loopTime = 0.
            roll = 0.
            pitch = 0.
            Xacc = 0.
            Yacc = 0.
            pass

        self.arduino.flushInput()

        return loopTime, Xacc, Yacc, roll, pitch

    def close(self):
        self.arduino.close()
