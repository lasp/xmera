# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

"""
Module Name:        simplePowerSink
"""

import inspect
import os

import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'xmera'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.utilities import unitTestSupport                  # general support file with common unit test functions
from xmera.simulation import simplePowerSink
from xmera.architecture import messaging
from xmera.utilities import macros



@pytest.mark.parametrize("function", ["defaultPowerSink"
                                      , "statusPowerSink"
                                      ])
def test_allTest_SimplePowerSink(show_plots, function):
    """Module Unit Test"""
    [testResults, testMessage] = eval(function + '()')
    assert testResults < 1, testMessage


def defaultPowerSink():
    """Module Unit Test"""
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTaskName, testProcessRate)

    testModule = simplePowerSink.SimplePowerSink()
    testModule.modelTag = "powerSink"
    testModule.nodePowerOut = 10.  # Watts
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    dataLog = testModule.nodePowerOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    drawData = dataLog.netPower

    # compare the module results to the truth values
    accuracy = 1e-16

    truePower = 10.0  # Module should be off

    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        [truePower]*3, drawData, accuracy, "powerSinkOutput",
        testFailCount, testMessages)

    if testFailCount:
        print("Failed test_default()")
    else:
        print("Passed")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


def statusPowerSink():
    """Module Unit Test"""
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTaskName, testProcessRate)

    testModule = simplePowerSink.SimplePowerSink()
    testModule.modelTag = "powerSink"
    testModule.nodePowerOut = 10.  # Watts
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # create the input messages
    powerStatusMsg = messaging.DeviceStatusMsgPayload()  # Create a structure for the input message
    powerStatusMsg.deviceStatus = 0
    powerMsg = messaging.DeviceStatusMsg().write(powerStatusMsg)
    testModule.nodeStatusInMsg.subscribeTo(powerMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = testModule.nodePowerOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    drawData = dataLog.netPower

    # compare the module results to the truth values
    accuracy = 1e-16

    truePower = 0.0  # Module should be off

    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        [truePower]*3, drawData, accuracy, "powerSinkStatusTest",
        testFailCount, testMessages)

    if testFailCount:
        print("Failed test_status()")
    else:
        print("Passed")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-alone python script
#
if __name__ == "__main__":
    test_allTest_SimplePowerSink()
    # test_default()
