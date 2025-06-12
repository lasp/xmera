#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import mrpSteering  # import the module that is to be tested
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
import numpy.testing


@pytest.mark.parametrize("K1", [0.15, 0])
@pytest.mark.parametrize("K3", [1, 0])
@pytest.mark.parametrize("omegaMax", [1.5 * macros.D2R, 0.001 * macros.D2R])
def test_mrp_steering_tracking(show_plots, K1, K3, omegaMax):
    r"""
    **Validation Test Description**

    This unit test  compares the computed :math:`\pmb\omega_{\mathcal{B}^{\ast}/\mathcal{R}}` and
    :math:`\pmb\omega_{\mathcal{B}^{\ast}/\mathcal{R}}'` to truth values computed in the python unit test.

    **Test Parameters**

    This test checks a set of gains ``K1``, ``K3`` and ``omegaMax`` on a rigid body with no external
    torques, and with a fixed input reference attitude message. The commanded rate solution
    is evaluated against python computed values at 0s, 0.5s and 1s to within a tolerance of :math:`10^{-12}`.

    :param show_plots: flag indicating if plots should be shown.
    :param K1: The control gain :math:`K_1`
    :param K3: The control gain :math:`K_3`
    :param omegaMax: The control gain :math:`\omega_{\text{max}}`
    :return: void

    """
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    module = mrpSteering.MrpSteering()
    module.modelTag = "mrpSteering"
    unitTestSim.AddModelToTask(unitTaskName, module)

    module.setK1(K1)
    module.setK3(K3)
    module.setOmegaMax(omegaMax)

    guidCmdData = messaging.AttGuidMsgPayload()
    sigma_BR = np.array([0.3, -0.5, 0.7])
    guidCmdData.sigma_BR = sigma_BR
    guidCmdData.omega_BR_B = np.array([0.010, -0.020, 0.015])
    guidCmdData.omega_RN_B = np.array([-0.02, -0.01, 0.005])
    guidCmdData.domega_RN_B = np.array([0.0002, 0.0003, 0.0001])
    guidInMsg = messaging.AttGuidMsg().write(guidCmdData)

    dataLog = module.rateCmdOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    module.guidInMsg.subscribeTo(guidInMsg)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    omegaAstTrue, omegaAstPTrue = findTrueValues(guidCmdData, module)

    numpy.testing.assert_allclose(dataLog.omega_BastR_B, omegaAstTrue, atol=1e-12)
    numpy.testing.assert_allclose(dataLog.omegap_BastR_B, omegaAstPTrue, atol=1e-12)




def findTrueValues(guidCmdData, module):

    omegaMax = module.getOmegaMax()
    sigma = np.asarray(guidCmdData.sigma_BR)
    K1 = np.asarray(module.getK1())
    K3 = np.asarray(module.getK3())
    Bmat = RigidBodyKinematics.BmatMRP(sigma)
    omegaAst = []   #np.asarray([0, 0, 0])
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
