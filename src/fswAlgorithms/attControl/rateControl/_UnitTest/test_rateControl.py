# SPDX-License-Identifier: ISC
# Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import numpy as np
import pytest

from xmera.utilities import SimulationBaseClass
from xmera.fswAlgorithms import rateControl
from xmera.utilities import macros
from xmera.architecture import messaging

@pytest.mark.parametrize("setExtTorque", [False, True])

def test_rateControl(setExtTorque):
    r"""
    **Validation Test Description**

    This test confirms the functionality of the rateControl fsw algorithm. The spacecraft inertia tensor message is set,
    as well as a guidance message. An external torque can be toggled on or off. The module is then run for a few time
    steps and the determined control torque output is compared to the computed truth value.

    **Test Parameters**

    The unit test verifies that the module output torque message vector matches expected values. The test
    method parameters include the following.

    :param setExtTorque: flag to set the knownTorquePntB_B variable
    :return: void
    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the rateControl module
    rateCntrl = rateControl.RateControl()
    rateCntrl.setDerivativeGainP(150.0)
    rateCntrl.modelTag = "rateControl"
    unitTestSim.AddModelToTask(unitTaskName, rateCntrl)

    # Set the external torque
    knownTorquePntB_B = np.array([0.0, 0.0, 0.0])
    if setExtTorque:
        knownTorquePntB_B = np.array([0.1, 0.2, 0.3])
        rateCntrl.setKnownTorquePntB_B(knownTorquePntB_B)

    # Create the attitude guidance input message
    attGuidanceMessageData = messaging.AttGuidMsgPayload()
    attGuidanceMessageData.sigma_BR = np.array([0.3, -0.5, 0.7])
    attGuidanceMessageData.omega_BR_B = np.array([0.010, -0.020, 0.015])
    attGuidanceMessageData.omega_RN_B = np.array([-0.02, -0.01, 0.005])
    attGuidanceMessageData.domega_RN_B = np.array([0.0002, 0.0003, 0.0001])
    attGuidanceMessage = messaging.AttGuidMsg().write(attGuidanceMessageData)

    # Create the vehicleConfig fsw message
    vehicleConfigIn = messaging.VehicleConfigMsgPayload()
    vehicleConfigIn.ISCPntB_B = [1000., 0., 0.,
                                  0., 800., 0.,
                                  0., 0., 800.]
    vcInMsg = messaging.VehicleConfigMsg().write(vehicleConfigIn)

    # Set up data logging
    cmdTorqueOutMsgDataLog = rateCntrl.cmdTorqueOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, cmdTorqueOutMsgDataLog)

    # Connect messages
    rateCntrl.vehConfigInMsg.subscribeTo(vcInMsg)
    rateCntrl.guidInMsg.subscribeTo(attGuidanceMessage)

    # Execute the simulation
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    # Extract logged data for test check
    cmdTorqueTruth = [findTrueTorques(rateCntrl, attGuidanceMessageData, vehicleConfigIn, knownTorquePntB_B)]*3

    # Compare the module results to the computed truth value
    accuracy = 1e-12
    np.testing.assert_allclose(cmdTorqueTruth,
                               cmdTorqueOutMsgDataLog.torqueRequestBody,
                               atol=accuracy,
                               rtol=0,
                               verbose=True)

def findTrueTorques(rateCntrl, attGuidanceMessageData, vehicleConfigOut, knownTorquePntB_B):
    P = rateCntrl.getDerivativeGainP()
    sigma_BR = np.array(attGuidanceMessageData.sigma_BR)
    omega_BR_B = np.array(attGuidanceMessageData.omega_BR_B)
    omega_RN_B = np.array(attGuidanceMessageData.omega_RN_B)
    domega_RN_B = np.array(attGuidanceMessageData.domega_RN_B)
    omega_BN_B = omega_BR_B + omega_RN_B

    ISCPntB_B = np.identity(3)
    ISCPntB_B[0][0] = vehicleConfigOut.ISCPntB_B[0]
    ISCPntB_B[1][1] = vehicleConfigOut.ISCPntB_B[4]
    ISCPntB_B[2][2] = vehicleConfigOut.ISCPntB_B[8]

    return (- P * omega_BR_B
            + np.cross(omega_RN_B, np.dot(ISCPntB_B, omega_BN_B))
            + np.dot(ISCPntB_B, domega_RN_B - np.cross(omega_BN_B, omega_RN_B))
            - knownTorquePntB_B)

if __name__ == "__main__":
    test_rateControl(False)
