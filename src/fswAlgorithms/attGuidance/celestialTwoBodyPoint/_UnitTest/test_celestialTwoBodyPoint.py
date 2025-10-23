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
from xmera.architecture import messaging
from xmera.fswAlgorithms import celestialTwoBodyPoint  # module that is to be tested
from xmera.utilities import RigidBodyKinematics as rbk
# Import all of the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.utilities import astroFunctions as af
from xmera.utilities import macros
from xmera.utilities import unitTestSupport  # general support file with common unit test functions
from numpy import linalg as la


def compute_celestial_two_body_point(R_P1, v_P1, a_P1, R_P2, v_P2, a_P2):

    # Beforehand computations
    R_n = np.cross(R_P1, R_P2)
    v_n = np.cross(v_P1, R_P2) + np.cross(R_P1, v_P2)
    a_n = np.cross(a_P1, R_P2) + np.cross(R_P1, a_P2) + 2 * np.cross(v_P1, v_P2)

    # Reference Frame generation
    r1_hat = R_P1/la.norm(R_P1)
    r3_hat = R_n/la.norm(R_n)
    r2_hat = np.cross(r3_hat, r1_hat)
    RN = np.array([r1_hat, r2_hat, r3_hat])
    sigma_RN = rbk.C2MRP(RN)

    # Reference base-vectors first time-derivative
    I_33 = np.identity(3)
    C1 = I_33 - np.outer(r1_hat, r1_hat)
    dr1_hat = 1.0 / la.norm(R_P1) * np.dot(C1, v_P1)
    C3 = I_33 - np.outer(r3_hat, r3_hat)
    dr3_hat = 1.0 / la.norm(R_n) * np.dot(C3, v_n)
    dr2_hat = np.cross(dr3_hat, r1_hat) + np.cross(r3_hat, dr1_hat)

    # Angular Velocity computation
    omega_RN_R = np.array([
        np.dot(r3_hat, dr2_hat),
        np.dot(r1_hat, dr3_hat),
        np.dot(r2_hat, dr1_hat)
    ])
    omega_RN_N = np.dot(RN.T, omega_RN_R)

    # Reference base-vectors second time-derivative
    temp33_1 = 2 * np.outer(dr1_hat, r1_hat) + np.outer(r1_hat, dr1_hat)
    ddr1_hat = 1.0 / la.norm(R_P1) * (np.dot(C1, a_P1) - np.dot(temp33_1, v_P1))
    temp33_3 = 2 * np.outer(dr3_hat, r3_hat) + np.outer(r3_hat, dr3_hat)
    ddr3_hat = 1.0 / la.norm(R_n) * (np.dot(C3, a_n) - np.dot(temp33_3, v_n))
    ddr2_hat = np.cross(ddr3_hat, r1_hat) + np.cross(ddr1_hat, r3_hat) + 2 * np.cross(dr3_hat, dr1_hat)

    # Angular Acceleration computation
    domega_RN_R = np.array([
        np.dot(dr3_hat, dr2_hat) + np.dot(r3_hat, ddr2_hat) - np.dot(omega_RN_R, dr1_hat),
        np.dot(dr1_hat, dr3_hat) + np.dot(r1_hat, ddr3_hat) - np.dot(omega_RN_R, dr2_hat),
        np.dot(dr2_hat, dr1_hat) + np.dot(r2_hat, ddr1_hat) - np.dot(omega_RN_R, dr3_hat)

    ])
    domega_RN_N = np.dot(RN.T, domega_RN_R)

    return sigma_RN, omega_RN_N, domega_RN_N


@pytest.mark.parametrize("secondary_body", [True, False])
def test_celestial_two_body_point_test_function(secondary_body):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    module = celestialTwoBodyPoint.CelestialTwoBodyPoint()
    module.modelTag = "celestialTwoBodyPoint"
    module.setSingularityThresh(1.0 * af.D2R)
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Previous Computation of Initial Conditions for the test
    a = af.E_radius * 2.8
    e = 0.0
    i = 0.0
    Omega = 0.0
    omega = 0.0
    f = 60 * af.D2R
    (r, v) = af.OE2RV(af.mu_E, a, e, i, Omega, omega, f)
    r_BN_N = np.array([0., 0., 0.])
    v_BN_N = np.array([0., 0., 0.])
    celPositionVec = r
    celVelocityVec = v

    # Navigation Input Message
    NavStateOutData = messaging.NavTransMsgPayload()  # Create a structure for the input message
    NavStateOutData.r_BN_N = r_BN_N
    NavStateOutData.v_BN_N = v_BN_N
    navMsg = messaging.NavTransMsg().write(NavStateOutData)

    # Spice Input Message of Primary Body
    CelBodyData = messaging.EphemerisMsgPayload()
    CelBodyData.r_BdyZero_N = celPositionVec
    CelBodyData.v_BdyZero_N = celVelocityVec
    celBodyMsg = messaging.EphemerisMsg().write(CelBodyData)

    dataLog = module.attRefOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    module.transNavInMsg.subscribeTo(navMsg)
    module.celBodyInMsg.subscribeTo(celBodyMsg)
    if secondary_body:
        SecBodyData = messaging.EphemerisMsgPayload()
        secPositionVec = [500., 500., 500.]
        SecBodyData.r_BdyZero_N = secPositionVec
        secVelocityVec = [100., -10., 20.]
        SecBodyData.v_BdyZero_N = secVelocityVec
        cel2ndBodyMsg = messaging.EphemerisMsg().write(SecBodyData)

        module.secCelBodyInMsg.subscribeTo(cel2ndBodyMsg)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    ## truth values
    a = af.E_radius * 2.8
    e = 0.0
    i = 0.0
    Omega = 0.0
    omega = 0.0
    f = 60 * af.D2R
    (r, v) = af.OE2RV(af.mu_E, a, e, i, Omega, omega, f)
    r_BN_N = np.array([0., 0., 0.])
    v_BN_N = np.array([0., 0., 0.])
    celPositionVec = r
    celVelocityVec = v

    R_P1 = celPositionVec - r_BN_N
    v_P1 = celVelocityVec - v_BN_N
    a_P1 = np.array([0., 0., 0.])
    if secondary_body:
        R_P2 = secPositionVec - r_BN_N
        v_P2 = secVelocityVec - v_BN_N
        a_P2 = np.array([0., 0., 0.])
    else:
        R_P2 = np.cross(R_P1, v_P1)
        v_P2 = np.cross(R_P1, a_P1)
        a_P2 = np.cross(v_P1, a_P1)

    sigma_RN, omega_RN_N, domega_RN_N = compute_celestial_two_body_point(R_P1, v_P1, a_P1, R_P2, v_P2, a_P2)

    true_sigma_RN = [sigma_RN] * 3
    true_omega_RN_N = [omega_RN_N] * 3
    true_domega_RN_N = [domega_RN_N] * 3

    ## module output
    sigma_RN = dataLog.sigma_RN
    omega_RN_N = dataLog.omega_RN_N
    domega_RN_N = dataLog.domega_RN_N

    # compare the module results to the truth values
    accuracy = 1e-12

    np.testing.assert_allclose(sigma_RN, true_sigma_RN, rtol=0, atol=accuracy, err_msg='sigma_RN', verbose=True)
    np.testing.assert_allclose(omega_RN_N, true_omega_RN_N, rtol=0, atol=accuracy, err_msg='omega_RN_N', verbose=True)
    np.testing.assert_allclose(domega_RN_N, true_domega_RN_N, rtol=0, atol=accuracy, err_msg='domega_RN_N', verbose=True)


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_celestial_two_body_point_test_function(True)
