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

import numpy as np
import pytest

from Basilisk.utilities import SimulationBaseClass
from Basilisk.fswAlgorithms import rateDamp
from Basilisk.utilities import macros
from Basilisk.architecture import messaging

@pytest.mark.parametrize("P", [0.1, 2.0])
def test_rateDamp(show_plots, P):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(1.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the rateDamp module
    attControl = rateDamp.RateDamp()
    attControl.setRateGain(P)
    attControl.modelTag = "rateDamp"
    unitTestSim.AddModelToTask(unitTaskName, attControl)

    # Create the rateDamp module input navigation message
    omega_BN_B = np.array([0.1, -0.2, 0.3])
    NavAttMessageData = messaging.NavAttMsgPayload()
    NavAttMessageData.omega_BN_B = omega_BN_B
    NavAttMsg = messaging.NavAttMsg().write(NavAttMessageData)
    attControl.attNavInMsg.subscribeTo(NavAttMsg)

    # Set up data logging
    cmdTorqueDataLog = attControl.cmdTorqueOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, cmdTorqueDataLog)

    # Run the simulation
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    # Extract the logged data
    cmdTorqueData = cmdTorqueDataLog.torqueRequestBody[0]

    # Compute the truth command torque
    truthCmdTorque = -attControl.getRateGain() * omega_BN_B

    # Check that the module-output command torque vector matches the computed truth value
    accuracy = 1e-12
    np.testing.assert_allclose(cmdTorqueData, truthCmdTorque, rtol=0, atol=accuracy, verbose=True)

if __name__ == "__main__":
    test_rateDamp(False, 1)
