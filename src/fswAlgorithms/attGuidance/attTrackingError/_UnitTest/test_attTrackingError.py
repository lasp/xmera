
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import numpy as np

from Basilisk.utilities import SimulationBaseClass
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.architecture import messaging

def test_attTrackingError(show_plots):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create instance of attTrackingError
    module = attTrackingError.AttTrackingError()
    module.modelTag = "attTrackingError"
    unitTestSim.AddModelToTask(unitTaskName, module)
    vector = [0.01, 0.05, -0.55]
    module.sigma_R0R = vector

    # Create navigation message
    NavStateOutData = messaging.NavAttMsgPayload()
    sigma_BN = [0.25, -0.45, 0.75]
    NavStateOutData.sigma_BN = sigma_BN
    omega_BN_B = [-0.015, -0.012, 0.005]
    NavStateOutData.omega_BN_B = omega_BN_B
    navStateInMsg = messaging.NavAttMsg().write(NavStateOutData)

    # Create reference frame message
    RefStateOutData = messaging.AttRefMsgPayload()
    sigma_RN = [0.35, -0.25, 0.15]
    RefStateOutData.sigma_RN = sigma_RN
    omega_RN_N = [0.018, -0.032, 0.015]
    RefStateOutData.omega_RN_N = omega_RN_N
    domega_RN_N = [0.048, -0.022, 0.025]
    RefStateOutData.domega_RN_N = domega_RN_N
    refInMsg = messaging.AttRefMsg().write(RefStateOutData)

    # Set up data logging
    dataLog = module.attGuidOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Connect messages
    module.attNavInMsg.subscribeTo(navStateInMsg)
    module.attRefInMsg.subscribeTo(refInMsg)

    # Run the simulation
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.3))
    unitTestSim.ExecuteSimulation()

    # Check sigma_BR
    moduleOutput = dataLog.sigma_BR[0]

    sigma_RN2 = rbk.addMRP(np.array(sigma_RN), -np.array(vector))
    RN = rbk.MRP2C(sigma_RN2)
    BN = rbk.MRP2C(np.array(sigma_BN))
    BR = np.dot(BN, RN.T)

    # Set the filtered output truth states
    trueVector = rbk.C2MRP(BR)

    # Compare the module results to the truth values
    accuracy = 1e-12
    np.testing.assert_allclose(trueVector, moduleOutput, atol=accuracy, verbose=True)

    # Check omega_BR_B
    moduleOutput = dataLog.omega_BR_B[0]

    # Set the filtered output truth states
    trueVector = np.array(omega_BN_B) - np.dot(BN, np.array(omega_RN_N))

    # Compare the module results to the truth values
    np.testing.assert_allclose(trueVector, moduleOutput, atol=accuracy, verbose=True)

    # Check omega_RN_B
    moduleOutput = dataLog.omega_RN_B[0]

    # Set the filtered output truth states
    trueVector = np.dot(BN, np.array(omega_RN_N))

    # Compare the module results to the truth values
    np.testing.assert_allclose(trueVector, moduleOutput, atol=accuracy, verbose=True)

    # Check domega_RN_B
    moduleOutput = dataLog.domega_RN_B[0]

    # Set the filtered output truth states
    trueVector = np.dot(BN, np.array(domega_RN_N))

    # Compare the module results to the truth values
    np.testing.assert_allclose(trueVector, moduleOutput, atol=accuracy, verbose=True)


if __name__ == "__main__":
    test_attTrackingError(False)
