# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

"""
Module Name:        rateMsgConverter
"""

import inspect
import os

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'xmera'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.utilities import unitTestSupport
from xmera.fswAlgorithms import rateMsgConverter
from xmera.utilities import macros
from xmera.architecture import messaging

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.


# update "module" in this function name to reflect the module name
def test_module(show_plots):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = rateMsgConvertFunction(show_plots)
    assert testResults < 1, testMessage


def rateMsgConvertFunction(show_plots):
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

    # Construct algorithm and associated C++ container
    module = rateMsgConverter.RateMsgConverter()
    module.modelTag = "rateMsgConverter"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = messaging.IMUSensorBodyMsgPayload()
    inputMessageData.AngVelBody = [-0.1, 0.2, -0.3]
    inMsg = messaging.IMUSensorBodyMsg().write(inputMessageData)
    module.imuRateInMsg.subscribeTo(inMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.navRateOutMsg.recorder()
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

    # compare the module results to the truth values
    accuracy = 1e-12
    print("accuracy = " + str(accuracy))

    # This pulls the actual data log from the simulation run.
    moduleOutput = dataLog.omega_BN_B
    # set the filtered output truth states
    trueVector = [
        [-0.1, 0.2, -0.3],
        [-0.1, 0.2, -0.3],
        [-0.1, 0.2, -0.3]
    ]
    testFailCount, testMessages = unitTestSupport.compareArray(trueVector, moduleOutput,
                                                               accuracy, "Output Vector",
                                                               testFailCount, testMessages)

    moduleOutput = dataLog.sigma_BN

    # set the filtered output truth states
    trueVector = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0]
    ]
    testFailCount, testMessages = unitTestSupport.compareArray(trueVector, moduleOutput,
                                                               accuracy, "Output MRP Vector",
                                                               testFailCount, testMessages)

    moduleOutput = dataLog.vehSunPntBdy

    # set the filtered output truth states
    trueVector = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0]
    ]
    testFailCount, testMessages = unitTestSupport.compareArray(trueVector, moduleOutput,
                                                               accuracy, "Output sun heading Vector",
                                                               testFailCount, testMessages)

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + module.modelTag)
    else:
        print("Failed: " + module.modelTag)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(False)
