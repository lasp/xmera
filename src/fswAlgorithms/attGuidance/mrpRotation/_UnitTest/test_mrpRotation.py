# ISC License
#
#  Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
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
#

import inspect
import os

import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

import numpy as np

from Basilisk.utilities import SimulationBaseClass
from Basilisk.fswAlgorithms import mrpRotation
from Basilisk.utilities import macros as mc
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.architecture import messaging


def compute_truth(sigma_RR0, omega_RR0_R, RefStateInData, dt, cmdStateFlag, testReset):

    ansSigma = []
    ansOmega_RN_N = []
    ansdOmega_RN_N = []

    sigma_R0N = RefStateInData.sigma_RN
    R0N = rbk.MRP2C(sigma_R0N)
    omega_R0N_N = RefStateInData.omega_RN_N
    domega_R0N_N = RefStateInData.domega_RN_N

    # compute 0th time step
    s0 = np.array(sigma_RR0)
    s1=rbk.addMRP(np.array(sigma_R0N), np.array(sigma_RR0))
    RR0 = rbk.MRP2C(sigma_RR0)
    RN = np.dot(RR0, R0N)

    omega_RR0_N = np.dot(RN.T, omega_RR0_R)
    omega_RN_N = omega_RR0_N + omega_R0N_N

    domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
    domega_RN_N = domega_RR0_N + domega_R0N_N

    ansSigma.append(s1.tolist())
    ansOmega_RN_N.append(omega_RN_N.tolist())
    ansdOmega_RN_N.append(domega_RN_N.tolist())
    ansSigma.append(s1.tolist())
    ansOmega_RN_N.append(omega_RN_N.tolist())
    ansdOmega_RN_N.append(domega_RN_N.tolist())

    # compute 1st time step
    B =  rbk.BmatMRP(sigma_RR0)
    sigma_RR0 += dt * 0.25 * np.dot(B, omega_RR0_R)
    RR0 = rbk.MRP2C(sigma_RR0)
    RN = np.dot(RR0, R0N)
    sigma_RN = rbk.C2MRP(RN)
    ansSigma.append(sigma_RN.tolist())

    omega_RR0_N = np.dot(RN.T, omega_RR0_R)
    omega_RN_N = omega_RR0_N + omega_R0N_N
    ansOmega_RN_N.append(omega_RN_N.tolist())

    domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
    domega_RN_N = domega_RR0_N + domega_R0N_N
    ansdOmega_RN_N.append(domega_RN_N.tolist())

    # compute 2nd time step
    B =  rbk.BmatMRP(sigma_RR0)
    sigma_RR0 += dt * 0.25 * np.dot(B, omega_RR0_R)
    RR0 = rbk.MRP2C(sigma_RR0)
    RN = np.dot(RR0, R0N)
    sigma_RN = rbk.C2MRP(RN)
    ansSigma.append(sigma_RN.tolist())

    omega_RR0_N = np.dot(RN.T, omega_RR0_R)
    omega_RN_N = omega_RR0_N + omega_R0N_N
    ansOmega_RN_N.append(omega_RN_N.tolist())

    domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
    domega_RN_N = domega_RR0_N + domega_R0N_N
    ansdOmega_RN_N.append(domega_RN_N.tolist())

    # Testing Reset function
    if testReset:
        if cmdStateFlag:
            sigma_RR0 = s0
        # compute 0th time step
        s1 = rbk.addMRP(np.array(sigma_R0N), np.array(sigma_RR0))
        RR0 = rbk.MRP2C(sigma_RR0)
        RN = np.dot(RR0, R0N)

        omega_RR0_N = np.dot(RN.T, omega_RR0_R)
        omega_RN_N = omega_RR0_N + omega_R0N_N

        domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
        domega_RN_N = domega_RR0_N + domega_R0N_N

        ansSigma.append(s1.tolist())
        ansOmega_RN_N.append(omega_RN_N.tolist())
        ansdOmega_RN_N.append(domega_RN_N.tolist())

        # compute 1st time step
        B = rbk.BmatMRP(sigma_RR0)
        sigma_RR0 += dt * 0.25 * np.dot(B, omega_RR0_R)
        RR0 = rbk.MRP2C(sigma_RR0)
        RN = np.dot(RR0, R0N)
        sigma_RN = rbk.C2MRP(RN)
        ansSigma.append(sigma_RN.tolist())

        omega_RR0_N = np.dot(RN.T, omega_RR0_R)
        omega_RN_N = omega_RR0_N + omega_R0N_N
        ansOmega_RN_N.append(omega_RN_N.tolist())

        domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
        domega_RN_N = domega_RR0_N + domega_R0N_N
        ansdOmega_RN_N.append(domega_RN_N.tolist())

    return ansSigma, ansOmega_RN_N, ansdOmega_RN_N


@pytest.mark.parametrize("cmdStateFlag", [False, True])
@pytest.mark.parametrize("testReset", [False, True])
# provide a unique test method name, starting with test_
def test_mrpRotation(show_plots, cmdStateFlag, testReset):
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Test times
    updateTime = 0.5     # update process rate update time
    totalTestSimTime = 1.5

    # Create test thread
    testProcessRate = mc.sec2nano(updateTime)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    module = mrpRotation.MrpRotation()
    module.modelTag = "mrpRotation"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Initialize the test module configuration data
    sigma_RR0 = np.array([0.3, .5, 0.0])
    module.setSigmaRR0(sigma_RR0)
    omega_RR0_R = np.array([0.1, 0.0, 0.0]) * mc.D2R
    module.setOmegaRR0(omega_RR0_R)

    if cmdStateFlag:
        desiredAtt = messaging.AttStateMsgPayload()
        sigma_RR0 = np.array([0.1, 0.0, -0.2])
        desiredAtt.state = sigma_RR0
        omega_RR0_R = np.array([0.1, 1.0, 0.5]) * mc.D2R
        desiredAtt.rate = omega_RR0_R
        desInMsg = messaging.AttStateMsg().write(desiredAtt)
        module.desiredAttInMsg.subscribeTo(desInMsg)

    # Reference Frame Message
    RefStateInData = messaging.AttRefMsgPayload()  # Create a structure for the input message
    sigma_R0N = np.array([0.1, 0.2, 0.3])
    RefStateInData.sigma_RN = sigma_R0N
    omega_R0N_N = np.array([0.1, 0.0, 0.0])
    RefStateInData.omega_RN_N = omega_R0N_N
    domega_R0N_N = np.array([0.0, 0.0, 0.0])
    RefStateInData.domega_RN_N = domega_R0N_N
    attRefMsg = messaging.AttRefMsg().write(RefStateInData)
    module.attRefInMsg.subscribeTo(attRefMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.attRefOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(mc.sec2nano(totalTestSimTime))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    if testReset:
        module.reset(1)
        unitTestSim.ConfigureStopTime(mc.sec2nano(totalTestSimTime+1.0))        # seconds to stop simulation
        unitTestSim.ExecuteSimulation()

    sigma_RN_true, omega_RN_true, dOmega_RN_true = compute_truth(sigma_RR0,omega_RR0_R,RefStateInData,updateTime, cmdStateFlag, testReset)

    accuracy = 1e-12

    np.testing.assert_allclose(dataLog.sigma_RN, sigma_RN_true, atol=accuracy, rtol=0, verbose=True)
    np.testing.assert_allclose(dataLog.omega_RN_N, omega_RN_true, atol=accuracy, rtol=0, verbose=True)
    np.testing.assert_allclose(dataLog.domega_RN_N, dOmega_RN_true, atol=accuracy, rtol=0, verbose=True)


if __name__ == "__main__":
    test_mrpRotation(False, False, True)
