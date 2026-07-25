# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

"""
Purpose:  Run the external pulsed torque disturance dynEffector
"""

import numpy as np
import pytest
from xmera.simulation import ExtPulsedTorque
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros
from xmera.utilities import unitTestSupport  # general support file with common unit test functions


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)


@pytest.mark.parametrize("offCount", [
      (3)
     ,(0)
])
# provide a unique test method name, starting with test_
def test_module(show_plots, offCount):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(
            show_plots, offCount)
    assert testResults < 1, testMessage


def run(show_plots, offCount):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"
    unitProcessName = "testProcess"

    scSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the dynamics simulation process
    #

    dynProcess = scSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    dynProcess.addTask(unitTaskName, macros.sec2nano(0.1))

    testObject = ExtPulsedTorque.ExtPulsedTorque()
    testObject.modelTag = "externalPulsedTorque"

    # These don't do anything. They are here to confirm they don't do anything
    testObject.readInputMessages()
    testObject.writeOutputMessages(0)

    testObject.pulsedTorqueExternalPntB_B = [[-1], [1],[ -1]]
    testObject.countOnPulse = 1
    testObject.countOff = offCount

    scSim.AddModelToTask(unitTaskName, testObject)

    #
    #   Setup data logging
    #
    testObjectLog = testObject.logger("torqueExternalPntB_B")
    scSim.AddModelToTask(unitTaskName, testObjectLog)

    #
    #   initialize the simulation
    #
    scSim.InitializeSimulation()

    #
    #   run the simulation
    #
    DT = 0.1
    testProcessRate = macros.sec2nano(DT)
    for tStop in range(1, 11):
        scSim.ConfigureStopTime(macros.sec2nano(tStop*DT))
        scSim.ExecuteSimulation()
        testObject.computeForceTorque(scSim.TotalSim.getCurrentNanos(), testProcessRate)
        scSim.TotalSim.singleStepProcesses()

    # log the data
    dataTorque = testObjectLog.torqueExternalPntB_B[1:,:]

    np.set_printoptions(precision=16)

    #
    #   set true position information
    #
    if (offCount == 3):
        trueTorque_B = [
                  [0.0, 0.0, 0.0]
                , [-1.0, 1.0, -1.0]
                , [1.0, -1.0, 1.0]
                , [0.0, 0.0, 0.0]
                , [0.0, 0.0, 0.0]
                , [0.0, 0.0, 0.0]
                , [-1.0, 1.0, -1.0]
                , [1.0, -1.0, 1.0]
                , [0.0, 0.0, 0.0]
                , [0.0, 0.0, 0.0]
                , [0.0, 0.0, 0.0]
        ]
    if (offCount == 0):
        trueTorque_B = [
                  [0.0, 0.0, 0.0]
                , [-1.0, 1.0, -1.0]
                , [1.0, -1.0, 1.0]
                , [-1.0, 1.0, -1.0]
                , [1.0, -1.0, 1.0]
                , [-1.0, 1.0, -1.0]
                , [1.0, -1.0, 1.0]
                , [-1.0, 1.0, -1.0]
                , [1.0, -1.0, 1.0]
                , [-1.0, 1.0, -1.0]
                , [1.0, -1.0, 1.0]
        ]

    # compare the module results to the truth values
    accuracy = 1e-12

    if (len(trueTorque_B) != len(dataTorque)):
        testFailCount += 1
        testMessages.append("FAILED:  ExtPulsedTorque failed torque unit test (unequal array sizes)\n")
    else:
        for i in range(0,len(trueTorque_B)):
            # check a vector values
            if not unitTestSupport.isArrayEqual(dataTorque[i],trueTorque_B[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  ExtPulsedTorque failed torque unit test at t=" + str(dataTorque[i,0]*macros.NANO2SEC) + "sec\n")

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(False,       # show_plots
                3            # offCount
               )
