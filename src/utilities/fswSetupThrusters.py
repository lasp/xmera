# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

#   FSW Setup Utilities for Thrusters
#

from xmera.architecture import messaging


thrList = []


def create(
        rThrust_B,
        tHatThrust_B,
        Fmax
    ):
    """
    This function is called to setup a FSW RW device in python, and adds it to the of RW
    devices in rwList[].  This list is accessible from the parent python script that
    imported this rw library script, and thus any particular value can be over-ridden
    by the user.

    Args:
        rThrust_B: position of thruster in spacecraft body frame
        tHatThrust_B: direction of thrust vector in B frame
        Fmax: maximum thrust force value

    """
    global thrList

    # create the blank Thruster object
    thrPointer = messaging.THRConfigMsgPayload()

    thrPointer.rThrust_B = rThrust_B
    thrPointer.tHatThrust_B = tHatThrust_B
    thrPointer.maxThrust = Fmax

    # add RW to the list of RW devices
    thrList.append(thrPointer)

    return


def writeConfigMessage():
    """
    This function should be called after all devices are created with create()
    It creates the C-class container for the array of RW devices, and attaches
    this container to the spacecraft object
    :return:
    """
    global thrList

    thrClass = messaging.THRArrayConfigMsgPayload()
    thrClass.thrusters = thrList
    thrClass.numThrusters = len(thrList)
    thrConfigInMsg = messaging.THRArrayConfigMsg().write(thrClass)
    thrConfigInMsg.this.disown()

    return thrConfigInMsg

def clearSetup():
    global thrList

    thrList = []

    return

def getNumOfDevices():
    return len(thrList)
