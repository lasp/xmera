# ISC License
#
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import inspect
import os

import numpy as np
import pytest

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.fswAlgorithms import rwMotorVoltage
from Basilisk.utilities import fswSetupRW
from Basilisk.utilities import macros
from Basilisk.architecture import messaging

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

@pytest.mark.parametrize("useLargeVoltage, useAvailability, useTorqueLoop", [(False, False, False),
                                                                             (True, False, False),
                                                                             (False, True, False),
                                                                             (False, False, True)])

def test_rw_motor_voltage(show_plots, useLargeVoltage, useAvailability, useTorqueLoop):
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    unitTestSim = SimulationBaseClass.SimBaseClass()

    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    module = rwMotorVoltage.RwMotorVoltage(1.0, 10.0)
    module.modelTag = "rwMotorVoltage"
    unitTestSim.AddModelToTask(unitTaskName, module)

    module.setVoltageRange(1.0, 11.0)  # test setter method

    if useTorqueLoop:
        module.setGainK(1.5)
        rwSpeedMessage = messaging.RWSpeedMsgPayload()
        rwSpeedMessage.wheelSpeeds = [1.0, 2.0, 1.5, -3.0]      # rad/sec Omega's
        rwSpeedInMsg = messaging.RWSpeedMsg().write(rwSpeedMessage)
        module.rwSpeedInMsg.subscribeTo(rwSpeedInMsg)

    # Create RW configuration parameter input message
    GsMatrix_B = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 1.0]
    ]  # the create routine below normalizes these vectors
    fswSetupRW.clearSetup()
    for i in range(4):
        fswSetupRW.create(GsMatrix_B[i],    #           spin axis
                          0.1,              # kg*m^2    J2
                          0.2)              # Nm        uMax
    rwConfigInMsg = fswSetupRW.writeConfigMessage()
    module.rwParamsInMsg.subscribeTo(rwConfigInMsg)
    numRW = fswSetupRW.getNumOfDevices()

    # Create RW motor torque input message
    usMessageData = messaging.RwMotorTorqueMsgPayload()
    if useLargeVoltage:
        usMessageData.motorTorque = [0.5, 0.0, -0.15, -0.5]           # [Nm] RW motor torque cmds
    else:
        usMessageData.motorTorque = [0.05, 0.0, -0.15, -0.2]  # [Nm] RW motor torque cmds
    rwMotorTorqueInMsg = messaging.RwMotorTorqueMsg().write(usMessageData)
    module.torqueInMsg.subscribeTo(rwMotorTorqueInMsg)

    # create RW availability message
    if useAvailability:
        rwAvailabilityMessage = messaging.RWAvailabilityMsgPayload()
        rwAvailArray = np.zeros(messaging.RW_EFF_CNT, dtype=int)
        rwAvailArray.fill(messaging.AVAILABLE)
        rwAvailArray[2] = messaging.UNAVAILABLE        # make 3rd RW unavailable
        rwAvailabilityMessage.wheelAvailability = rwAvailArray
        rwAvailInMsg = messaging.RWAvailabilityMsg().write(rwAvailabilityMessage)
        module.rwAvailInMsg.subscribeTo(rwAvailInMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.voltageOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    if useTorqueLoop:
        rwSpeedMessage.wheelSpeeds = [1.1, 2.1, 1.1, -4.1]  # rad/sec Omega's
        rwSpeedInMsg.write(rwSpeedMessage)

    unitTestSim.ConfigureStopTime(macros.sec2nano(1.5))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # reset the module to test this functionality
    module.reset(1)     # this module reset function needs a time input (in NanoSeconds)

    # run the module again for an additional 1.0 seconds
    unitTestSim.ConfigureStopTime(macros.sec2nano(3.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    voltage = dataLog.voltage[:, :numRW]

    # set the filtered output truth states
    voltageTrue=[]
    if not useLargeVoltage and not useAvailability and not useTorqueLoop:
        voltageTrue = [
                   [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                   ]
    if useLargeVoltage and not useAvailability and not useTorqueLoop:
        voltageTrue = [
                   [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                   ]
    if not useLargeVoltage and useAvailability and not useTorqueLoop:
        voltageTrue = [
                   [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                   ]
    if not useLargeVoltage and not useAvailability and useTorqueLoop:
        voltageTrue = [
                   [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [5.75, -2.5, -11., -9.5]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [7.25, 0., -11., -11.]
                   ]

    # compare the module results to the truth values
    accuracy = 1e-10

    np.testing.assert_allclose(voltage, voltageTrue, atol=accuracy, rtol=0, verbose=True)


if __name__ == "__main__":
    test_rw_motor_voltage(False,
                          False,
                          False,
                          True,
                          "Four")
