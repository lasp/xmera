# SPDX-License-Identifier: ISC
# Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import matplotlib.pyplot as plt
import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fswAlgorithms import mrpSteering_C  # import the module that is to be tested
from xmera.fswAlgorithms import rateServoFullNonlinear_C
from xmera.utilities import RigidBodyKinematics
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros
from xmera.utilities import unitTestSupport  # general support file with common unit test functions


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

@pytest.mark.parametrize("K1", [0.15, 0])
@pytest.mark.parametrize("K3", [1, 0])
@pytest.mark.parametrize("omegaMax", [1.5 * macros.D2R, 0.001])


def test_mrp_steering_tracking(show_plots,K1, K3, omegaMax):
    r"""
    **Validation Test Description**

    This unit test is an integrated test of this module with :ref:`rateServoFullNonlinear` as well,
    comparing the desired torques computed :math:`{\bf L}_r` with truth values computed in the test.

    **Test Parameters**

    This test checks a set of gains ``K1``, ``K3`` and ``omegaMax`` on a rigid body with no external
    torques, and with a fixed input reference attitude message. The commanded rate solution
    is evaluated against python computed values at 0s, 0.5s, 1.0s, 1.5s and 2s to within a
    tolerance of :math:`10^{-12}`.

    :param show_plots: flag indicating if plots should be shown.
    :param K1: The control gain :math:`K_1`
    :param K3: The control gain :math:`K_3`
    :param omegaMax: The control gain :math:`\omega_{\text{max}}`
    :return: void

    """
    [testResults, testMessage] = mrp_steering_tracking(show_plots,K1, K3, omegaMax)
    assert testResults < 1, testMessage


def mrp_steering_tracking(show_plots,K1, K3, omegaMax):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    module = mrpSteering_C.MrpSteering_C()
    module.modelTag = "mrpSteering"

    servo = rateServoFullNonlinear_C.RateServoFullNonlinear_C()
    servo.modelTag = "rate_servo"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)
    unitTestSim.AddModelToTask(unitTaskName, servo)

    module.K1 = K1
    module.K3 = K3
    module.omega_max = omegaMax
    servo.Ki = 0.01
    servo.P = 150.0
    servo.integralLimit = 2. / servo.Ki * 0.1
    servo.knownTorquePntB_B = [0., 0., 0.]

    #   Create input message and size it because the regular creator of that message
    #   is not part of the test.
    #   attGuidOut Message:
    guidCmdData = messaging.AttGuidMsgPayload()  # Create a structure for the input message
    guidCmdData.sigma_BR = [0.3, -0.5, 0.7]
    guidCmdData.omega_BR_B = [0.010, -0.020, 0.015]
    guidCmdData.omega_RN_B = [-0.02, -0.01, 0.005]
    guidCmdData.domega_RN_B = [0.0002, 0.0003, 0.0001]
    guidInMsg = messaging.AttGuidMsg().write(guidCmdData)

    # vehicleConfigData Message:
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    I = [1000., 0., 0.,
         0., 800., 0.,
         0., 0., 800.]
    vehicleConfigOut.ISCPntB_B = I
    vcInMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # wheelSpeeds Message
    rwSpeedMessage = messaging.RWSpeedMsgPayload()
    Omega = [10.0, 25.0, 50.0, 100.0]
    rwSpeedMessage.wheelSpeeds = Omega
    rwInMsg = messaging.RWSpeedMsg().write(rwSpeedMessage)

    # wheelConfigData message
    def writeMsgInWheelConfiguration():
        rwConfigParams = messaging.RWArrayConfigMsgPayload()
        rwConfigParams.GsMatrix_B = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.5773502691896258, 0.5773502691896258, 0.5773502691896258
        ]
        rwConfigParams.JsList = [0.1, 0.1, 0.1, 0.1]
        rwConfigParams.numRW = 4
        msg = messaging.RWArrayConfigMsg().write(rwConfigParams)
        jsList = rwConfigParams.JsList
        GsMatrix_B = rwConfigParams.GsMatrix_B
        return jsList, GsMatrix_B, msg

    jsList, GsMatrix_B, rwParamInMsg = writeMsgInWheelConfiguration()

    # wheelAvailability message
    rwAvailList = []
    rwAvailabilityMessage = messaging.RWAvailabilityMsgPayload()
    rwAvail = [messaging.AVAILABLE, messaging.AVAILABLE, messaging.AVAILABLE, messaging.AVAILABLE]
    rwAvailabilityMessage.wheelAvailability = rwAvail
    rwAvailInMsg = messaging.RWAvailabilityMsg().write(rwAvailabilityMessage)
    rwAvailList.append(rwAvail)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = servo.cmdTorqueOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # connect messages
    module.guidInMsg.subscribeTo(guidInMsg)
    servo.guidInMsg.subscribeTo(guidInMsg)
    servo.vehConfigInMsg.subscribeTo(vcInMsg)
    servo.rwParamsInMsg.subscribeTo(rwParamInMsg)
    servo.vehConfigInMsg.subscribeTo(vcInMsg)
    servo.rwSpeedsInMsg.subscribeTo(rwInMsg)
    servo.rateSteeringInMsg.subscribeTo(module.rateCmdOutMsg)
    servo.rwAvailInMsg.subscribeTo(rwAvailInMsg)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    servo.reset(1)  # this module reset function needs a time input (in NanoSeconds)

    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # Compute true values
    trueVals = findTrueTorques(module, servo, guidCmdData, rwSpeedMessage, vehicleConfigOut, rwAvailList)

    # set the filtered output truth states
    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0, len(trueVals)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(dataLog.torqueRequestBody[i], trueVals[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + module.modelTag + " Module failed torqueRequestBody unit test at t="
                                + str(dataLog.times[i] * macros.NANO2SEC) + "sec \n")


    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
    if show_plots:
        plt.show()

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + module.modelTag)

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


def findTrueValues(guidCmdData, module):

    omegaMax = module.omega_max
    sigma = np.asarray(guidCmdData.sigma_BR)
    K1 = np.asarray(module.K1)
    K3 = np.asarray(module.K3)
    Bmat = RigidBodyKinematics.BmatMRP(sigma)
    omegaAst = []#np.asarray([0, 0, 0])
    omegaAst_P = []

    for i in range(len(sigma)):
        steerRate = -1*(2*omegaMax/np.pi)*np.arctan((K1*sigma[i]+K3*sigma[i]*sigma[i]*sigma[i])*np.pi/(2*omegaMax))
        omegaAst.append(steerRate)
    #print omegaAst


    if 1:#module.ignoreOuterLoopFeedforward: #should be "if not"
        sigmaP = 0.25*Bmat.dot(omegaAst)
        for i in range(len(sigma)):
            omegaAstRate = (K1+3*K3*sigma[i]**2)/(1+((K1*sigma[i]+K3*sigma[i]**3)**2)*(np.pi/(2*omegaMax))**2)*sigmaP[i]
            omegaAst_P.append(-omegaAstRate)
    else:
        omegaAst_P = np.asarray([0, 0, 0])

    return omegaAst, omegaAst_P

def findTrueTorques(module,servo, guidCmdData,rwSpeedMessage,vehicleConfigOut, rwAvailMsg):
    Lr = []

    #Read in variables
    numRW = servo.rwConfigParams.numRW
    L = np.asarray(servo.knownTorquePntB_B)
    steps = [0, 0, .5, 0, .5]
    omega_BR_B = np.asarray(guidCmdData.omega_BR_B)
    omega_RN_B = np.asarray(guidCmdData.omega_RN_B)
    omega_BN_B = omega_BR_B + omega_RN_B #find body rate
    domega_RN_B = np.asarray(guidCmdData.domega_RN_B)


    omega_BastR_B, omegap_BastR_B = findTrueValues(guidCmdData, module)

    omega_BastN_B = omega_BastR_B+omega_RN_B
    omega_BBast_B = omega_BN_B - omega_BastN_B

    Isc = np.asarray(vehicleConfigOut.ISCPntB_B)
    Isc = np.reshape(Isc, (3, 3))
    Ki = servo.Ki
    P = servo.P
    jsVec = servo.rwConfigParams.JsList[0:numRW]
    #GsMatrix_B_array = np.asarray(GsMatrix)
    GsMatrix = (servo.rwConfigParams.GsMatrix_B)
    GsMatrix_B_array = np.reshape(GsMatrix[0:numRW * 3], (numRW, 3))

    #Compute toruqes
    for i in range(len(steps)):
        dt = steps[i]
        if dt == 0:
            zVec = np.asarray([0, 0, 0])

        #evaluate integral term
        if Ki > 0 and abs(servo.integralLimit) > 0: #if integral feedback is on
            zVec = dt * omega_BBast_B + zVec  # z = integral(del_omega)
            # Make sure each component is less than the integral limit
            for i in range(3):
                if zVec[i] > servo.integralLimit:
                        zVec[i] = zVec[i]/abs(zVec[i])*servo.integralLimit

        else: #integral gain turned off/negative setting
            zVec = np.asarray([0, 0, 0])

        #compute torque Lr
        Lr0 = Ki * zVec  # +K*sigmaBR
        Lr1 = Lr0 + P * omega_BBast_B  # +P*deltaOmega

        GsHs = np.array([0,0,0])

        if numRW > 0:
            for i in range(numRW):
                if rwAvailMsg[0][i] == 0:  # Make RW availability check
                    GsHs = GsHs + np.dot(GsMatrix_B_array[i, :], jsVec[i]*(np.dot(omega_BN_B, GsMatrix_B_array[i, :]) + rwSpeedMessage.wheelSpeeds[i]))
                    # J_s*(dot(omegaBN_B,Gs_vec)+Omega_wheel)

        Lr2 = Lr1 - np.cross(omega_BastN_B, (Isc.dot(omega_BN_B)+GsHs))  #  - omega_BastN x ([I]omega + [Gs]h_s)

        Lr3 = Lr2 - Isc.dot(omegap_BastR_B + domega_RN_B - np.cross(omega_BN_B, omega_RN_B))
        # - [I](d(omega_B^ast/R)/dt + d(omega_r)/dt - omega x omega_r)
        Lr4 = Lr3 + L
        Lr4 = -Lr4
        Lr.append(np.ndarray.tolist(Lr4))
    return Lr

if __name__ == "__main__":
    test_mrp_steering_tracking(False, 0.15, 1.0, 0.025)
