import numpy as np
import pytest

from xmera.utilities import SimulationBaseClass
from xmera.fswAlgorithms import mrpPD
from xmera.utilities import macros
from xmera.architecture import messaging

@pytest.mark.parametrize("setExtTorque", [False, True])
def test_mrpPD(show_plots, setExtTorque):
    r"""
    **Validation Test Description**

    The unit test for this module verifies that the module output control torque vector matches the expected value.
    Given the spacecraft vehicle configuration input message containing the spacecraft inertia, an input attitude
    guidance message, and an optional known external torque vector, the module-computed control torque command vector
    is logged and compared with the computed truth value.

    **Test Parameters**

    The unit test verifies that the module output control torque vector matches the expected value. The test
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
    mrp_pd = mrpPD.MrpPD()
    mrp_pd.modelTag = "mrpPD"
    mrp_pd.setDerivativeGainP(150.0)
    mrp_pd.setProportionalGainK(0.15)
    knownTorquePntB_B = np.array([0.0, 0.0, 0.0])
    if setExtTorque:
        knownTorquePntB_B = np.array([0.1, 0.2, 0.3])
        mrp_pd.setKnownTorquePntB_B(knownTorquePntB_B)
    unitTestSim.AddModelToTask(unitTaskName, mrp_pd)

    # Create the mrpPD module attitude guidance input message
    guidCmdData = messaging.AttGuidMsgPayload()
    guidCmdData.sigma_BR = np.array([0.3, -0.5, 0.7])
    guidCmdData.omega_BR_B = np.array([0.010, -0.020, 0.015])  # [rad/s]
    guidCmdData.omega_RN_B = np.array([-0.02, -0.01, 0.005])  # [rad/s]
    guidCmdData.domega_RN_B = np.array([0.0002, 0.0003, 0.0001])  # [rad/s^2]
    guidInMsg = messaging.AttGuidMsg().write(guidCmdData)
    mrp_pd.guidInMsg.subscribeTo(guidInMsg)

    # Create the mrpPD module vehicle configuration input FSW message:
    ISCPntB_B = [1000., 0., 0., 0., 800., 0., 0., 0., 800.]  # [kg*m^2]
    vehicleConfigIn = messaging.VehicleConfigMsgPayload()
    vehicleConfigIn.ISCPntB_B = ISCPntB_B
    vcInMsg = messaging.VehicleConfigMsg().write(vehicleConfigIn)
    mrp_pd.vehConfigInMsg.subscribeTo(vcInMsg)

    # Set up data logging
    cmdTorqueDataLog = mrp_pd.cmdTorqueOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, cmdTorqueDataLog)

    # Run the simulation for 3*process rate, 4 total steps including zero
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    # Compute the truth control torque vector
    truthTorque = findTrueTorques(mrp_pd, guidCmdData, np.array(ISCPntB_B).reshape(3, 3), knownTorquePntB_B)  # [Nm]

    # Compare the module-computed command torque to the truth value
    accuracy = 1e-12
    np.testing.assert_allclose(truthTorque,
                               cmdTorqueDataLog.torqueRequestBody[-1],
                               atol=accuracy,
                               rtol=0,
                               verbose=True)

def findTrueTorques(mrp_pd, guidCmdData, ISCPntB_B, knownTorquePntB_B):
    # Compute hub inertial angular velocity in B-frame components
    omega_BR_B = np.array(guidCmdData.omega_BR_B)
    omega_RN_B = np.array(guidCmdData.omega_RN_B)
    omega_BN_B = omega_BR_B + omega_RN_B

    K = mrp_pd.getProportionalGainK()
    P = mrp_pd.getDerivativeGainP()
    sigma_BR = np.array(guidCmdData.sigma_BR)
    domega_RN_B = np.array(guidCmdData.domega_RN_B)

    # Compute required attitude control torque
    Lr = (- K * sigma_BR - P * omega_BR_B + np.cross(omega_RN_B, ISCPntB_B @ omega_BN_B)
          + ISCPntB_B @ (domega_RN_B - np.cross(omega_BN_B, omega_RN_B)) - knownTorquePntB_B)  # [Nm]

    return Lr

if __name__ == "__main__":
    test_mrpPD(False, False)
