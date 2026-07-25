# SPDX-License-Identifier: ISC
# Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from xmera.utilities import SimulationBaseClass, macros
from xmera.fswAlgorithms import ephemDifference
from xmera.utilities import astroFunctions
from xmera.architecture import messaging

@pytest.mark.parametrize("ephBdyCount", [3, 0])
def test_ephemDifference(ephBdyCount):
    """ Test ephemDifference. """
    ephemDifferenceTestFunction(ephBdyCount)

def ephemDifferenceTestFunction(ephBdyCount):
    """ Test the ephemDifference module. Setup a simulation, """

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTaskName, testProcessRate)  # Add a new task to the process

    ephemDiff = ephemDifference.EphemDifference()

    # This calls the algContain to setup the selfInit, update, and reset
    ephemDiff.modelTag = "ephemDifference"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, ephemDiff)

    # Create the input message.
    inputEphemBase = messaging.EphemerisMsgPayload() # The clock correlation message ?
    # Get the Earth's position and velocity
    position, velocity = astroFunctions.Earth_RV(astroFunctions.JulianDate([2018, 10, 16]))
    inputEphemBase.r_BdyZero_N = position
    inputEphemBase.v_BdyZero_N = velocity
    inputEphemBase.timeTag = 1234.0
    ephBaseInMsg = messaging.EphemerisMsg().write(inputEphemBase)
    ephemDiff.ephBaseInMsg.subscribeTo(ephBaseInMsg)
    functions = [astroFunctions.Mars_RV, astroFunctions.Jupiter_RV, astroFunctions.Saturn_RV]

    ephInMsgList = list()
    dataLogList = list()
    if ephBdyCount == 3:
        for i in range(ephBdyCount):
            # Create the change body message
            changeBodyMsg = ephemDifference.EphemChangeConfig()

            # Create the input message to the change body config
            inputMsg = messaging.EphemerisMsgPayload()
            position, velocity = functions[i](astroFunctions.JulianDate([2018, 10, 16]))
            inputMsg.r_BdyZero_N = position
            inputMsg.v_BdyZero_N = velocity
            inputMsg.timeTag = 321.0

            # Set this message
            msg = messaging.EphemerisMsg().write(inputMsg)
            ephInMsgList.append(msg)
            changeBodyMsg.ephInMsg.subscribeTo(msg)
            ephemDiff.changeBodies[i].ephInMsg.subscribeTo(msg)

            # Hook up a recorder to the existing output message
            rec = ephemDiff.changeBodies[i].ephOutMsg.recorder()
            dataLogList.append(rec)
            unitTestSim.AddModelToTask(unitTaskName, rec)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()
    # The result isn't going to change with more time. The module will continue to produce the same result
    unitTestSim.ConfigureStopTime(0)  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    if ephBdyCount == 3:
        trueRVector = [[69313607.6209608,  -75620898.04028425,   -5443274.17030424],
                       [-5.33462105e+08,  -7.56888610e+08,   1.17556184e+07],
                       [9.94135029e+07,  -1.54721593e+09,   1.65081472e+07]]

        trueVVector = [[15.04232523,  -1.13359121,   0.47668898],
                       [23.2531093,  -33.17628299,  -0.22550391],
                       [21.02793499, -25.86425597,  -0.38273815]]

        for i in range(ephBdyCount):

            outputData_R = dataLogList[i].r_BdyZero_N
            outputData_V = dataLogList[i].v_BdyZero_N
            timeTag = dataLogList[i].timeTag

            np.testing.assert_allclose(0, abs(trueRVector[i] - outputData_R), atol=10)

            np.testing.assert_allclose(0, abs(trueVVector[i] - outputData_V), atol=1e-4)

            np.testing.assert_equal(321.0, timeTag[0], err_msg="ephemDifference timeTag output body " + str(i))

    np.testing.assert_equal(ephBdyCount, ephemDiff.ephBdyCount, err_msg="input/output message count is wrong.")


if __name__ == '__main__':
    test_ephemDifference(3)
