# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
#

import math


#   function to convert seconds to an integer nanoseconds value
def sec2nano(time):
    """convert seconds to nano-seconds"""
    return int(time*1E9+0.5)


#   function to convert minutes to an integer nanoseconds value
def min2nano(time):
    "convert minutes to nano-seconds"
    return int(time*1E9*60 + 0.5)


#   function to convert hours to an integer nanoseconds value
def hour2nano(time):
    """convert hours to nano-seconds"""
    return int(time*1E9*60*60 + 0.5)


#   function to convert days to an integer nanoseconds value
def day2nano(time):
    """convert days to nano-seconds"""
    return int(time*1E9*60*60*24 + 0.5)


#   variable to convert nano-seconds to seconds
NANO2SEC = 1E-9
NANO2MIN = (1./60.*1E-9)
NANO2HOUR = (1./60./60.*1E-9)

#   variable to convert degrees to radians
D2R = (math.pi/180.)

#   variable to convert radians to degrees
R2D = (180./math.pi)

#   variable to convert RPM to radians per second
RPM = (2.*math.pi/60.)

#   variable to convert RPM to radians per second
rpm2radsec = 2.0 * math.pi/60.
