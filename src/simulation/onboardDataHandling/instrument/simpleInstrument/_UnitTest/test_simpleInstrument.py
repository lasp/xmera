# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

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
from xmera.simulation import simpleInstrument
from xmera.architecture import messaging
from xmera.utilities import macros



@pytest.mark.parametrize("function", ["checkDefault"
                                      , "checkStatus"
                                      ])
def test_simpleInstrumentAll(show_plots, function):
    """Module Unit Test"""
    [testResults, testMessage] = eval(function + '()')
    assert testResults < 1, testMessage


def checkDefault():
    """
    **Validation Test Description**

    1. Whether the simpleInstrument provides the right output message (baudRate) while on;
    2. Whether the simpleInstrument provides the right output message (baudRate) while off.

    :param show_plots: Not used; no plots to be shown.

    :return:
    """

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

    testModule = simpleInstrument.SimpleInstrument()
    testModule.modelTag = "instrument1"
    testModule.nodeBaudRate = 9600. # baud
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    dataLog = testModule.nodeDataOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    generatedData = dataLog.baudRate

    # compare the module results to the truth values
    accuracy = 1e-16

    trueData = 9600.  # Module should be on

    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        [trueData]*3, generatedData, accuracy, "dataOutput",
        testFailCount, testMessages)

    if testFailCount:
        print(testMessages)
    else:
        print("Passed")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


def checkStatus():
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTaskName, testProcessRate)

    testModule = simpleInstrument.SimpleInstrument()
    testModule.modelTag = "instrument1"
    testModule.nodeBaudRate = 9600. # baud
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # create the input messages
    dataCmdMsg = messaging.DeviceCmdMsgPayload()  # Create a structure for the input message
    dataCmdMsg.deviceCmd = 0
    statMsg = messaging.DeviceCmdMsg().write(dataCmdMsg)
    testModule.nodeStatusInMsg.subscribeTo(statMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = testModule.nodeDataOutMsg.recorder()
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
    drawData = dataLog.baudRate

    # compare the module results to the truth values
    accuracy = 1e-16

    trueData = 0.0  # Module should be off

    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        [trueData]*3, drawData, accuracy, "instrumentStatusTest",
        testFailCount, testMessages)

    if testFailCount:
        print(testMessages)
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
    # checkDefault()
    checkStatus()
