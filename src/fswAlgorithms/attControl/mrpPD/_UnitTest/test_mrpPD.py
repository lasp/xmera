#
#  ISC License
#
#  Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
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
import numpy as np
import os
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk.utilities import SimulationBaseClass
from Basilisk.fswAlgorithms import mrpPD
from Basilisk.utilities import macros
from Basilisk.architecture import messaging

@pytest.mark.parametrize("setExtTorque", [False, True])
def test_mrp_PD_tracking(show_plots, setExtTorque):
    r"""
    **Validation Test Description**

    The unit test  for this module is kept as there are no branching code segments to account for different cases.
    The spacecraft inertia tensor message is setup, as well as a guidance message.  The module is then run for a
    few time steps and the control torque output message compared to a known answer.  The simulation only variable
    is if the known external torque is specified, or if the zero default vector is used.

    **Test Parameters**

    The unit test verifies that the module output torque message vector matches expected values.  The test
    method parameters include the following.

    :param show_plots: flag to show the test run plots
    :param setExtTorque: flag to set the knownTorquePntB_B variable
    :return: void
    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # Update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the mrpPD module
    module = mrpPD.MrpPD()
    module.modelTag = "mrpPD"
    module.setDerivativeGainP(150.0)
    module.setProportionalGainK(0.15)
    knownTorquePntB_B = np.array([0.0, 0.0, 0.0])
    if setExtTorque:
        knownTorquePntB_B = np.array([0.1, 0.2, 0.3])
        module.setKnownTorquePntB_B(knownTorquePntB_B)
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Create the mrpPD module attitude guidance input message
    guidCmdData = messaging.AttGuidMsgPayload()
    guidCmdData.sigma_BR = np.array([0.3, -0.5, 0.7])
    guidCmdData.omega_BR_B = np.array([0.010, -0.020, 0.015])  # [rad/s]
    guidCmdData.omega_RN_B = np.array([-0.02, -0.01, 0.005])  # [rad/s]
    guidCmdData.domega_RN_B = np.array([0.0002, 0.0003, 0.0001])  # [rad/s^2]
    guidInMsg = messaging.AttGuidMsg().write(guidCmdData)
    module.guidInMsg.subscribeTo(guidInMsg)

    # Create the mrpPD module vehicle configuration input FSW message:
    vehicleConfigIn = messaging.VehicleConfigMsgPayload()
    vehicleConfigIn.ISCPntB_B = [1000., 0., 0.,
                                  0., 800., 0.,
                                  0., 0., 800.]
    vcInMsg = messaging.VehicleConfigMsg().write(vehicleConfigIn)
    module.vehConfigInMsg.subscribeTo(vcInMsg)

    # Set up data logging
    dataLog = module.cmdTorqueOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Run the simulation for 3*process rate, 4 total steps including zero
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    # Compute the truth control torque vector
    trueVector = [findTrueTorques(module, guidCmdData, vehicleConfigIn, knownTorquePntB_B)]*3

    # Compare the module result to the truth value
    accuracy = 1e-12
    np.testing.assert_allclose(trueVector,
                               dataLog.torqueRequestBody,
                               atol=accuracy,
                               verbose=True)

def findTrueTorques(module, guidCmdData, vehicleConfigOut, knownTorquePntB_B):
    sigma_BR = np.array(guidCmdData.sigma_BR)
    omega_BR_B = np.array(guidCmdData.omega_BR_B)
    omega_RN_B = np.array(guidCmdData.omega_RN_B)
    domega_RN_B = np.array(guidCmdData.domega_RN_B)

    I = np.identity(3)
    I[0][0] = vehicleConfigOut.ISCPntB_B[0]
    I[1][1] = vehicleConfigOut.ISCPntB_B[4]
    I[2][2] = vehicleConfigOut.ISCPntB_B[8]

    K = module.getProportionalGainK()
    P = module.getDerivativeGainP()
    L = knownTorquePntB_B

    omega_BN_B = omega_BR_B + omega_RN_B
    temp1 = np.dot(I, omega_BN_B)
    temp2 = domega_RN_B - np.cross(omega_BN_B, omega_RN_B)

    Lr = K * sigma_BR + P * omega_BR_B - np.cross(omega_RN_B, temp1) - np.dot(I, temp2)
    Lr += L
    Lr *= -1.0

    return Lr

if __name__ == "__main__":
    test_mrp_PD_tracking(False, False)
