#
#  ISC License
#
#  Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
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
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import mrpSteering  # import the module that is to be tested
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

@pytest.mark.parametrize("K1", [0.15, 0])
@pytest.mark.parametrize("K3", [1.0, 0])
@pytest.mark.parametrize("omegaMax", [1.5 * macros.D2R, 0.001 * macros.D2R])

def test_mrp_steering_tracking(show_plots, K1, K3, omegaMax):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    module = mrpSteering.MrpSteering()
    module.modelTag = "mrpSteering"
    unitTestSim.AddModelToTask(unitTaskName, module)

    module.setK1(K1)
    module.setK3(K3)
    module.setOmegaMax(omegaMax)

    guidCmdData = messaging.AttGuidMsgPayload()  # Create a structure for the input message
    sigma_BR = np.array([0.3, -0.5, 0.7])
    guidCmdData.sigma_BR = sigma_BR
    omega_BR_B = np.array([0.010, -0.020, 0.015])
    guidCmdData.omega_BR_B = omega_BR_B
    omega_RN_B = np.array([-0.02, -0.01, 0.005])
    guidCmdData.omega_RN_B = omega_RN_B
    domega_RN_B = np.array([0.0002, 0.0003, 0.0001])
    guidCmdData.domega_RN_B = domega_RN_B
    guidInMsg = messaging.AttGuidMsg().write(guidCmdData)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.rateCmdOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # connect messages
    module.guidInMsg.subscribeTo(guidInMsg)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # Compute truth states
    omegaAstTrue, omegaAstPTrue = findTrueValues(guidCmdData, module)

    # compare the module results to the truth values
    accuracy = 1e-12

    np.testing.assert_allclose(dataLog.omega_BastR_B, omegaAstTrue, atol=accuracy, rtol=0, verbose=True)
    np.testing.assert_allclose(dataLog.omegap_BastR_B, omegaAstPTrue, atol=accuracy, rtol=0, verbose=True)


def findTrueValues(guidCmdData, module):

    omegaMax = module.getOmegaMax()
    sigma = np.asarray(guidCmdData.sigma_BR)
    K1 = np.asarray(module.getK1())
    K3 = np.asarray(module.getK3())
    Bmat = RigidBodyKinematics.BmatMRP(sigma)
    omegaAst = []
    omegaAst_P = []

    for i in range(len(sigma)):
        steerRate = -1*(2*omegaMax/np.pi)*np.arctan((K1*sigma[i]+K3*sigma[i]*sigma[i]*sigma[i])*np.pi/(2*omegaMax))
        omegaAst.append(steerRate)


    if 1:   #module.ignoreOuterLoopFeedforward: #should be "if not"
        sigmaP = 0.25*Bmat.dot(omegaAst)
        for i in range(len(sigma)):
            omegaAstRate = (K1+3*K3*sigma[i]**2)/(1+((K1*sigma[i]+K3*sigma[i]**3)**2)*(np.pi/(2*omegaMax))**2)*sigmaP[i]
            omegaAst_P.append(-omegaAstRate)
    else:
        omegaAst_P = np.asarray([0, 0, 0])

    omegaAst = [omegaAst, omegaAst, omegaAst]
    omegaAst_P = [omegaAst_P, omegaAst_P, omegaAst_P]

    return omegaAst, omegaAst_P


if __name__ == "__main__":
    test_mrp_steering_tracking(False, 0.1, 1.0, 1.0)
