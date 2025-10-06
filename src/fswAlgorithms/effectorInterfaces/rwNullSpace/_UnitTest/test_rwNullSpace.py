#
#  ISC License
#
#  Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

import inspect
import os

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import rwNullSpace
from Basilisk.utilities import SimulationBaseClass, macros
from numpy.linalg import inv

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

@pytest.mark.parametrize("numWheels, defaultDesired", [(3, True),
                                                       (4, True),
                                                       (3, False),
                                                       (4, False)])


def test_rwNullSpace(numWheels, defaultDesired):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))  # Add a new task to the process

    # Construct the rwNullSpace module
    module = rwNullSpace.RwNullSpace()
    module.setOmegaGain(.5) # The feedback gain value applied for the RW despin control law
    module.modelTag = "rwNullSpace"
    unitTestSim.AddModelToTask(unitTaskName, module)

    numRW = numWheels

    inputRWConstellationMsg = messaging.RWConstellationMsgPayload()
    inputRWConstellationMsg.numRW = numRW

    # Initialize the msg that gives the speed of the reaction wheels
    inputSpeedMsg = messaging.RWSpeedMsgPayload()

    if defaultDesired:
        desiredOmega = [0]*numRW
    else:
        desiredOmega = [5]*numRW
        inputDesiredSpeedMsg = messaging.RWSpeedMsgPayload()
        inputDesiredSpeedMsg.wheelSpeeds = desiredOmega

    gsHat = [[1, 0, 0], [0,1,0], [0, 0, 1]]
    if numWheels == 4:
        gs4Hat = np.array([1,1,1])
        gs4Hat = gs4Hat/np.sqrt(gs4Hat.dot(gs4Hat))
        gsHat.append(gs4Hat.tolist())

    # Iterate over all of the reaction wheels, create a rwConfigElementFswMsg, and add them to the rwConstellationFswMsg
    rwConfigElementList = list()
    for rw in range(numRW):
        rwConfigElementMsg = messaging.RWConfigElementMsgPayload()
        rwConfigElementMsg.gsHat_B = gsHat[rw] # Spin axis unit vector of the wheel in structure
        rwConfigElementMsg.Js = 0.08 # Spin axis inertia of wheel [kgm2]
        rwConfigElementMsg.uMax = 0.2 # maximum RW motor torque [Nm]

        # Add this to the list
        rwConfigElementList.append(rwConfigElementMsg)

    rwSpeeds = [10, 20, 30] # [rad/sec] The current angular velocities of the RW wheel
    if numWheels == 4:
        rwSpeeds.append(40)  # [rad/sec]
    inputSpeedMsg.wheelSpeeds = rwSpeeds

    # Set the array of the reaction wheels in RWConstellationFswMsg to the list created above
    inputRWConstellationMsg.reactionWheels = rwConfigElementList

    inputRWCmdMsg = messaging.RwMotorTorqueMsgPayload()
    usControl = [0.1, 0.2, 0.15] # [Nm] RW motor torque array
    if numWheels == 4:
        usControl.append(-0.2) # [Nm]
    inputRWCmdMsg.motorTorque = usControl

    # Set these messages
    rwSpeedMsg = messaging.RWSpeedMsg().write(inputSpeedMsg)
    rwConfigMsg = messaging.RWConstellationMsg().write(inputRWConstellationMsg)
    rwCmdMsg = messaging.RwMotorTorqueMsg().write(inputRWCmdMsg)

    dataLog = module.rwMotorTorqueOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # connect messages
    module.rwMotorTorqueInMsg.subscribeTo(rwCmdMsg)
    module.rwSpeedsInMsg.subscribeTo(rwSpeedMsg)
    module.rwConfigInMsg.subscribeTo(rwConfigMsg)
    if not defaultDesired:
        rwDesiredMsg = messaging.RWSpeedMsg().write(inputDesiredSpeedMsg)
        module.rwDesiredSpeedsInMsg.subscribeTo(rwDesiredMsg)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    #   Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    motorTorque = dataLog.motorTorque[:, :numRW]

    if numWheels == 3:
        # in this case there is no nullspace of the RW configuration.  The output torque should be the input torque
        trueVector = [inputRWCmdMsg.motorTorque[:numRW]]
    elif numWheels == 4:
        # in this case there is a 1D nullspace of [Gs]
        GsT = np.array(gsHat)
        Gs = GsT.transpose()
        tmp = Gs.dot(GsT)
        tmp = GsT.dot(inv(tmp))
        tmp = tmp.dot(Gs)
        tau = np.identity(numWheels) - tmp
        d = - module.getOmegaGain() * (np.array(rwSpeeds) - np.array(desiredOmega))
        uNull = tau.dot(d)
        trueTorque = np.array(usControl) + uNull
        trueVector = [trueTorque.tolist()]

    motorTorqueTrue = trueVector * 5


    accuracy = 1e-12
    np.testing.assert_allclose(motorTorque, motorTorqueTrue, atol=accuracy, rtol=0, verbose=True)


if __name__ == '__main__':
    test_rwNullSpace(4, True)
