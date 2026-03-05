# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import numpy
from xmera.architecture import messaging

#
#   FSW Setup Utilities for RW
#
rwList = []


def create(
        gsHat_B,
        Js,
        uMax = numpy.nan
    ):
    """
    Create a FSW RW object

    This function is called to setup a FSW RW device in python, and adds it to the of RW
    devices in rwList[].  This list is accessible from the parent python script that
    imported this rw library script, and thus any particular value can be over-ridden
    by the user.
    """
    global rwList

    # create the blank RW object
    RW = messaging.RWConfigElementMsgPayload()

    norm = numpy.linalg.norm(gsHat_B)
    if norm > 1e-10:
        gsHat_B = gsHat_B / norm
    else:
        print('Error: RW gsHat input must be non-zero 3x1 vector')
        exit(1)

    RW.gsHat_B = gsHat_B
    RW.uMax = uMax
    RW.Js = Js

    # add RW to the list of RW devices
    rwList.append(RW)

    return


def writeConfigMessage():
    """
    Write FSW RW array msg

    This function should be called after all devices are created with create()
    It creates the C-class container for the array of RW devices, and attaches
    this container to the spacecraft object

    """
    global rwList

    GsMatrix_B = []
    JsList = []
    uMaxList = []
    for rw in rwList:
        GsMatrix_B.extend(rw.gsHat_B)
        JsList.extend([rw.Js])
        uMaxList.extend([rw.uMax])

    rwConfigParams = messaging.RWArrayConfigMsgPayload()
    rwConfigParams.GsMatrix_B = GsMatrix_B
    rwConfigParams.JsList = JsList
    rwConfigParams.uMax = uMaxList
    rwConfigParams.numRW = len(rwList)
    rwConfigMsg = messaging.RWArrayConfigMsg().write(rwConfigParams)
    rwConfigMsg.this.disown()

    return rwConfigMsg


def clearSetup():
    global rwList

    rwList = []

    return


def getNumOfDevices():
    return len(rwList)
