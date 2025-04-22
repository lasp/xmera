
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
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    module = attTrackingError.AttTrackingError()
    module.modelTag = "attTrackingError"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    vector = [0.01, 0.05, -0.55]
    module.sigma_R0R = vector

    #
    # Navigation Message
    #
    NavStateOutData = messaging.NavAttMsgPayload()  # Create a structure for the input message
    sigma_BN = [0.25, -0.45, 0.75]
    NavStateOutData.sigma_BN = sigma_BN
    omega_BN_B = [-0.015, -0.012, 0.005]
    NavStateOutData.omega_BN_B = omega_BN_B
    navStateInMsg = messaging.NavAttMsg().write(NavStateOutData)

    #
    # Reference Frame Message
    #
    RefStateOutData = messaging.AttRefMsgPayload()  # Create a structure for the input message
    sigma_RN = [0.35, -0.25, 0.15]
    RefStateOutData.sigma_RN = sigma_RN
    omega_RN_N = [0.018, -0.032, 0.015]
    RefStateOutData.omega_RN_N = omega_RN_N
    domega_RN_N = [0.048, -0.022, 0.025]
    RefStateOutData.domega_RN_N = domega_RN_N
    refInMsg = messaging.AttRefMsg().write(RefStateOutData)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.attGuidOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # connect messages
    module.attNavInMsg.subscribeTo(navStateInMsg)
    module.attRefInMsg.subscribeTo(refInMsg)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.3))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    #
    # check sigma_BR
    #
    moduleOutput = dataLog.sigma_BR[0]

    sigma_RN2 = rbk.addMRP(np.array(sigma_RN), -np.array(vector))
    RN = rbk.MRP2C(sigma_RN2)
    BN = rbk.MRP2C(np.array(sigma_BN))
    BR = np.dot(BN, RN.T)
    # set the filtered output truth states
    trueVector = rbk.C2MRP(BR)

    # compare the module results to the truth values
    accuracy = 1e-12
    np.testing.assert_allclose(trueVector, moduleOutput, atol=accuracy, verbose=True)



    #
    # check omega_BR_B
    #
    moduleOutput = dataLog.omega_BR_B[0]

    # set the filtered output truth states
    trueVector = np.array(omega_BN_B) - np.dot(BN, np.array(omega_RN_N))

    # compare the module results to the truth values
    np.testing.assert_allclose(trueVector, moduleOutput, atol=accuracy, verbose=True)

    #
    # check omega_RN_B
    #
    moduleOutput = dataLog.omega_RN_B[0]

    # set the filtered output truth states
    trueVector = np.dot(BN, np.array(omega_RN_N))

    # compare the module results to the truth values
    np.testing.assert_allclose(trueVector, moduleOutput, atol=accuracy, verbose=True)

    #
    # check domega_RN_B
    #
    moduleOutput = dataLog.domega_RN_B[0]

    # set the filtered output truth states
    trueVector = np.dot(BN, np.array(domega_RN_N))

    # compare the module results to the truth values
    np.testing.assert_allclose(trueVector, moduleOutput, atol=accuracy, verbose=True)

    # Note that we can continue to step the simulation however we feel like.
    # Just because we stop and query data does not mean everything has to stop for good
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.6))    # run an additional 0.6 seconds
    unitTestSim.ExecuteSimulation()


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_attTrackingError(False)
