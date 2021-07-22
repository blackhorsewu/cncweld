#!/usr/bin/env python

from os import error
import rospy
import serial
import time

class CNC:
    __default_speed__ = 300 # mm per minute, 5mm per second

    def __init__(self, limits=[0, 0, 0], speed=__default_fspeed__):
        self.s = None              # s is the serial port
        self.abs_move = None       # absolute movement?
        self.defaultSpeed = speed  # feed speed
        self.pos = [0,0,0]         # current position of end effector
        self.origin = [0,0,0]      # Minimum coordinates
        self.limits = list(limits) # maximum coordinates

    def startup(self, port, baud=115200):
        self.s = serial.Serial(port, baud)
        time.sleep(2)
        self.s.write("$H\n") # Home the machine before anything
        self.s.write("$X\n") # unlock the machine after homing
        self.set_origin()    # set the Home position as the origin

    def shutdown(self):
        self.s.close()

    def position(self):
        # return a list [x, y, z] of the position of the end effector
        return list(self.pos)

    def setSpeed(self, speed):
        self.defaultSpeed = speed

    def home(self):
        # return to Home position
        self.s.write("$H\n")
        self.s.readline()
        self.pos = list(self.origin)

    def moveTo(self, x=None, y=None, z=None, speed=None, blockUntilComplete=True):
        # move to an absolute position, and return when movement completes
        if x is None and y is None and z is None: return
        if speed is None: speed = self.defaultSpeed

        self.ensureMovementMode(absoluteMode=True)

        gcode = 'G1'            # the G code is G1
        letters = 'xyz'         # the x, y, z axes name
        pos = (x, y, z)         # the x, y, z coordinate values

        # this does not seem to be right! self.pos is the current position before moving
        # pos - just constructed from the arguments is the new position
        newpos = list(self.pos) # this will be the target position move to

        # create gcode string and update position list for each argument that
        # is not None

        for i in range(3): # only 3 axes - x, y, z
            if pos[i] is not None:
                # check against limits
                if pos[i] < 0 or pos[i] >= self.limits[i]:
                    error
                    print(self.limits[i] + '=' + pos[i] + ' position outside limit\n')
                    return
                
                gcode += ' ' + letters[i] + str(pos[i]) # a space follow by the axes follow by the position on that axis
                newpos[i] = pos[i]

        gcode += ' f' + str(speed) + '\n' # followed by the feed speed

        self.s.write(gcode) # send the G code out to GRBL
        self.s.readline()   # wait to read a response from GRBL

        # update position of the End Effector if success
        # Check to make sure it is actually a success - see if the line read back is "ok"
        self.pos = newpos


