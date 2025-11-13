import numpy as np
import pytest

from xmera.utilities import SimulationBaseClass
from xmera.fswAlgorithms import sunSafePoint
from xmera.architecture import messaging
from xmera.utilities import macros as mc

@pytest.mark.parametrize("case", [
     (1)        # sun is visible, vectors are not aligned
    ,(2)        # sun is not visible, vectors are not aligned
    ,(3)        # sun is visible, vectors are aligned
    ,(4)        # sun is visible, vectors are oppositely aligned
    ,(5)        # sun is visible, vectors are oppositely aligned, and command sc is b1
    ,(6)        # sun is not visible, vectors are not aligned, no specified omega_RN_B value
    ,(7)        # sun is visible, vectors not aligned, nominal spin rate specified about sun heading vector
])

def test_sunSafePoint(show_plots, case):

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = mc.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the sunSafePoint module
    sun_safe_point = sunSafePoint.SunSafePoint()
    sun_safe_point.modelTag = "sunSafePoint"
    unitTestSim.AddModelToTask(unitTaskName, sun_safe_point)

    # Initialize sunSafePoint module configuration data
    sun_safe_point.setMinUnitMag(0.1)
    sun_safe_point.setSmallAngle(0.01 * mc.D2R)

    sHat_Cmd_B = []
    sunVec_B = []
    if case == 1:  # Sun visible, vectors not aligned
        sHat_Cmd_B = np.array([0.0, 0.0, 1.0])
        sunVec_B = np.array([1.0, 1.0, 0.0])

    elif case == 2:  # Sun not visible, search rate specified
        sHat_Cmd_B = np.array([0.0, 0.0, 1.0])
        sunVec_B = np.array([0.0, sun_safe_point.getMinUnitMag() / 2, 0.0])

        omega_RN_B_Search = np.array([0.0, 0.0, 0.1])
        sun_safe_point.setOmega_RN_B(omega_RN_B_Search)

    elif case == 3:  # Sun visible, vectors aligned
        sHat_Cmd_B = np.array([0.0, 0.0, 1.0])
        sunVec_B = sHat_Cmd_B

    elif case == 4:  # Sun visible, vectors oppositely aligned
        sHat_Cmd_B = np.array([0.0, 0.0, 1.0])
        sunVec_B = -sHat_Cmd_B

    elif case == 5:  # Sun visible, vectors oppositely aligned, sHatCmd is along b1
        sHat_Cmd_B = np.array([1.0, 0.0, 0.0])
        sunVec_B = -sHat_Cmd_B

    elif case == 6:  # Sun not visible, no search rate specified
        sHat_Cmd_B = np.array([0.0, 0.0, 1.0])
        sunVec_B = np.array([0.0, sun_safe_point.getMinUnitMag() / 2, 0.0])

    else:  # Sun visible, spin rate about sun heading vector specified
        sHat_Cmd_B = np.array([0.0, 0.0, 1.0])
        sunVec_B = np.array([1.0, 1.0, 0.0])

        sun_safe_point.setSunAxisSpinRate(1.5*mc.D2R)
        omega_RN_B_Search = sunVec_B/np.linalg.norm(sunVec_B) * sun_safe_point.getSunAxisSpinRate()

    sun_safe_point.setSHatBdyCmd(sHat_Cmd_B)

    # Create sunSafePoint sun direction input messages
    inputSunVecData = messaging.NavAttMsgPayload()
    inputSunVecData.vehSunPntBdy = sunVec_B
    sunInMsg = messaging.NavAttMsg().write(inputSunVecData)

    # Create sunSafePoint IMU input message
    inputIMUData = messaging.NavAttMsgPayload()
    omega_BN_B = np.array([0.01, 0.50, -0.2])
    inputIMUData.omega_BN_B = omega_BN_B
    imuInMsg = messaging.NavAttMsg().write(inputIMUData)

    # Set up data logging
    attGuidOutMsgDataLog = sun_safe_point.attGuidanceOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, attGuidOutMsgDataLog)

    # Connect messages
    sun_safe_point.sunDirectionInMsg.subscribeTo(sunInMsg)
    sun_safe_point.imuInMsg.subscribeTo(imuInMsg)

    # Run the simulation
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(mc.sec2nano(1.))
    sun_safe_point.reset(0)
    unitTestSim.ExecuteSimulation()

    # Check sigma_BR
    # Set the filtered output truth states
    if case == 1 or case == 7:
        eHat = np.cross(sunVec_B, sHat_Cmd_B)
        eHat = eHat / np.linalg.norm(eHat)
        Phi = np.arccos(np.dot(sunVec_B / np.linalg.norm(sunVec_B), sHat_Cmd_B))
        sigmaTrue = eHat * np.tan(Phi / 4.0)
        sigma_BRTruth = [
                    sigmaTrue.tolist(),
                    sigmaTrue.tolist(),
                    sigmaTrue.tolist()
                   ]
    if case == 2 or case == 3 or case == 6:
        sigma_BRTruth = [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ]
    if case == 4:
        eHat = np.cross(sHat_Cmd_B, np.array([1, 0, 0]))
        eHat = eHat / np.linalg.norm(eHat)
        Phi = np.arccos(np.dot(sunVec_B / np.linalg.norm(sunVec_B), sHat_Cmd_B))
        sigmaTrue = eHat * np.tan(Phi / 4.0)
        sigma_BRTruth = [
                    sigmaTrue.tolist(),
                    sigmaTrue.tolist(),
                    sigmaTrue.tolist()
               ]
    if case == 5:
        eHat = np.cross(sHat_Cmd_B, np.array([0, 1, 0]))
        eHat = eHat / np.linalg.norm(eHat)
        Phi = np.arccos(np.dot(sunVec_B / np.linalg.norm(sunVec_B), sHat_Cmd_B))
        sigmaTrue = eHat * np.tan(Phi / 4.0)
        sigma_BRTruth = [
            sigmaTrue.tolist(),
            sigmaTrue.tolist(),
            sigmaTrue.tolist()
        ]

    # Compare the module results to the truth values
    accuracy = 1e-12
    np.testing.assert_allclose(sigma_BRTruth,
                               attGuidOutMsgDataLog.sigma_BR,
                               atol=accuracy,
                               verbose=True)

    # Check omega_BR_B
    # Set the filtered output truth states
    if case == 1 or case == 3 or case == 4 or case == 5 or case == 6:
        omega_BR_BTruth = [
            omega_BN_B.tolist(),
            omega_BN_B.tolist(),
            omega_BN_B.tolist()
        ]
    if case == 2 or case == 7:
        omega_BR_BTruth = [
            (omega_BN_B - omega_RN_B_Search).tolist(),
            (omega_BN_B - omega_RN_B_Search).tolist(),
            (omega_BN_B - omega_RN_B_Search).tolist()
        ]

    # Compare the module results to the truth values
    np.testing.assert_allclose(omega_BR_BTruth,
                               attGuidOutMsgDataLog.omega_BR_B,
                               atol=accuracy,
                               verbose=True)

    # Check omega_RN_B
    # Set the filtered output truth states
    if case == 1 or case == 3 or case == 4 or case == 5 or case == 6:
        omega_RN_BTruth = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]
        ]
    if case == 2 or case == 7:
        omega_RN_BTruth = [
            omega_RN_B_Search,
            omega_RN_B_Search,
            omega_RN_B_Search
        ]

    # Compare the module results to the truth values
    np.testing.assert_allclose(omega_RN_BTruth,
                               attGuidOutMsgDataLog.omega_RN_B,
                               atol=accuracy,
                               verbose=True)

    # Check domega_RN_B
    # Set the filtered output truth states
    domega_RN_BTruth = [
               [0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0]
               ]

    # Compare the module results to the truth values
    np.testing.assert_allclose(domega_RN_BTruth,
                               attGuidOutMsgDataLog.domega_RN_B,
                               atol=accuracy,
                               verbose=True)

if __name__ == "__main__":
    test_sunSafePoint(False, 1)
