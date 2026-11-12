# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
#

import math

import numpy as np


def ns_per(unit):
    """Nanoseconds in one ``unit``, a numpy timedelta64 unit code such as "s", "m", "h" or "D"."""
    return np.timedelta64(1, unit) / np.timedelta64(1, "ns")


#   function to convert seconds to an integer nanoseconds value
def sec2nano(time):
    """convert seconds to nano-seconds"""
    return int(time*ns_per("s") + 0.5)


#   function to convert minutes to an integer nanoseconds value
def min2nano(time):
    "convert minutes to nano-seconds"
    return int(time*ns_per("m") + 0.5)


#   function to convert hours to an integer nanoseconds value
def hour2nano(time):
    """convert hours to nano-seconds"""
    return int(time*ns_per("h") + 0.5)


#   function to convert days to an integer nanoseconds value
def day2nano(time):
    """convert days to nano-seconds"""
    return int(time*ns_per("D") + 0.5)


#   variable to convert nano-seconds to seconds
NANO2SEC = 1.0 / ns_per("s")
NANO2MIN = 1.0 / ns_per("m")
NANO2HOUR = 1.0 / ns_per("h")

#   variable to convert degrees to radians
D2R = (math.pi/180.)

#   variable to convert radians to degrees
R2D = (180./math.pi)

#   variable to convert RPM to radians per second
RPM = (2.*math.pi/60.)

#   variable to convert RPM to radians per second
rpm2radsec = 2.0 * math.pi/60.
