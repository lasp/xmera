# SPDX-License-Identifier: ISC
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import pytest
import numpy as np



# Import all the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.fswAlgorithms import triad
from xmera.utilities import macros
from xmera.architecture import messaging


def SPE_angle(v1, v2):

    dot_product = np.dot(v1, v2)
    cross_product = np.cross(v1, v2)

    # Compute angle in degrees
    angle = np.degrees(np.arccos(np.clip(dot_product / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1.0, 1.0)))
    # Adjust to [0, 360] range based on cross product sign
    if cross_product.any() < 0:
        angle = 360 - angle
    return angle


def true_triad(eh_N,sh_N,a1_B,h1_B):
    r2=h1_B
    r3= np.cross(a1_B,h1_B)/np.linalg.norm(np.cross(a1_B,h1_B))
    r1=np.cross(r2,r3)

    n2= eh_N
    n1 = np.cross(sh_N,eh_N)/np.linalg.norm(np.cross(sh_N,eh_N))
    n3= np.cross(n1,n2)

    ND = (np.vstack((n1, n2, n3))).T
    RD = (np.vstack((r1, r2, r3))).T

    RN=RD@ND.T
    return RN

@pytest.mark.parametrize("case", [
    1         # SPE below 90 degrees, x-axis orthogonal to sun
    ,2        # SPE below 90 degrees, y-axis aligned to earth
    ,3        # SPE above 90 degrees, x-axis orthogonal to sun
    ,4        # SPE above 90 degrees, y-axis aligned to earth
])
def test_triadTestFunction(show_plots,case):

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    if case ==1 or case ==2 :

    # SPE = 7.16
        sh_N = [2.12024926e+11 ,2.12239088e+11, 6.60583756e-01]
        r_EN_N = [3.43401015e+11 ,2.76561597e+11, 2.78825040e+10]
    if case == 3 or case ==4:
        # SPE = 129.23
        sh_N = [-7.47993852e+10 ,-3.03274801e+08 ,-6.16397545e-01]
        r_EN_N = [5.65767033e+10 ,6.40192339e+10, 2.78825040e+10]

    # Sun direction in inertial coordinates
    sh_N = sh_N / np.linalg.norm(sh_N)
    r_BN_N = np.array([0,0,0])

    # earth direction in inertial coordinates
    eh_N = r_EN_N - r_BN_N
    r_EN_N = r_EN_N / np.linalg.norm(r_EN_N)
    eh_N = eh_N / np.linalg.norm(eh_N)


    SPE = SPE_angle(sh_N,eh_N)
    print(f"SPE: {SPE}")
    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    module = triad.Triad()
    module.modelTag = "triad"

    a1_B= np.array([1, 0 ,0] )# X axis
    h1_B= np.array([0, 1 ,0]) # Y axis

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)
    module.setA1Hat_B( a1_B)
    module.setH1Hat_B( h1_B)


# Create input navigation message
    sigma_BN = np.array([0.1, -0.2, 0.1])
    BN = rbk.MRP2C(sigma_BN)
    rS_B = np.matmul(BN, sh_N)
    NavAttMessageData = messaging.NavAttMsgPayload()
    NavAttMessageData.sigma_BN = sigma_BN
    NavAttMessageData.vehSunPntBdy = rS_B
    NavAttMsg = messaging.NavAttMsg().write(NavAttMessageData)
    module.attNavInMsg.subscribeTo(NavAttMsg)

    transNavData = messaging.NavTransMsgPayload()
    transNavData.r_BN_N =r_BN_N
    transNavMsg = messaging.NavTransMsg().write(transNavData)
    module.transNavInMsg.subscribeTo(transNavMsg)

    ephemerisData = messaging.EphemerisMsgPayload()
    ephemerisData.r_BdyZero_N = r_EN_N
    ephemerisMsg = messaging.EphemerisMsg().write(ephemerisData)
    module.ephemerisInMsg.subscribeTo(ephemerisMsg)



    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.attRefOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    moduleOutput = dataLog.sigma_RN
    sigma_RN = moduleOutput[0]
    RN = rbk.MRP2C(sigma_RN)


    check = -1
    if case == 1 or case == 3:
        check= np.dot(np.array(RN[0,:]),sh_N)
    if case == 2 or case == 4:
        check = np.dot(RN[1,:],eh_N)

    if case == 1 or case == 3:
        np.testing.assert_(check.round(3) == 0.0, msg=f"SPE angel: {SPE},  X-axis is not orthogonal to sun")
    elif case == 2 or case == 4:
        np.testing.assert_(check.round(3) == 1.0, msg=f"SPE angel: {SPE}, Y-axis aligned to earth")

if __name__ == "__main__":
    test_triadTestFunction(False,1)
