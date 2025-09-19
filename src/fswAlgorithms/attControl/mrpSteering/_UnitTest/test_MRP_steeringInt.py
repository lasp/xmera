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
from Basilisk.fswAlgorithms import rateServoFullNonlinear
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

@pytest.mark.parametrize("K1", [0.15, 0])
@pytest.mark.parametrize("K3", [1.0, 0])
@pytest.mark.parametrize("omegaMax", [1.5 * macros.D2R, 0.001])

def test_mrp_steering_tracking_integrated(show_plots, K1, K3, omegaMax):
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

    servo = rateServoFullNonlinear.RateServoFullNonlinear()
    servo.modelTag = "rate_servo"

    unitTestSim.AddModelToTask(unitTaskName, module)
    unitTestSim.AddModelToTask(unitTaskName, servo)

    module.setK1(K1)
    module.setK3(K3)
    module.setOmegaMax(omegaMax)

    servo.Ki = 0.01
    servo.P = 150.0
    servo.integralLimit = 2. / servo.Ki * 0.1
    servo.knownTorquePntB_B = [0., 0., 0.]

    # attGuidOut Message:
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
    rwConfigParams = messaging.RWArrayConfigMsgPayload()
    rwConfigParams.GsMatrix_B = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        0.5773502691896258, 0.5773502691896258, 0.5773502691896258
    ]
    rwConfigParams.JsList = [0.1, 0.1, 0.1, 0.1]
    rwConfigParams.numRW = 4
    rwParamInMsg = messaging.RWArrayConfigMsg().write(rwConfigParams)

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

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    servo.reset(1)  # this module reset function needs a time input (in NanoSeconds)

    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # Compute true values
    trueVals = findTrueTorques(module, servo, guidCmdData, rwSpeedMessage, vehicleConfigOut, rwAvailList)

    # compare the module results to the truth values
    accuracy = 1e-12

    np.testing.assert_allclose(dataLog.torqueRequestBody, trueVals, atol=accuracy, rtol=0, verbose=True)


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
    GsMatrix = (servo.rwConfigParams.GsMatrix_B)
    GsMatrix_B_array = np.reshape(GsMatrix[0:numRW * 3], (numRW, 3))

    # Compute toruqes
    for i in range(len(steps)):
        dt = steps[i]
        if dt == 0:
            zVec = np.asarray([0, 0, 0])

        # evaluate integral term
        if Ki > 0 and abs(servo.integralLimit) > 0: #if integral feedback is on
            zVec = dt * omega_BBast_B + zVec  # z = integral(del_omega)
            # Make sure each component is less than the integral limit
            for i in range(3):
                if zVec[i] > servo.integralLimit:
                        zVec[i] = zVec[i]/abs(zVec[i])*servo.integralLimit

        else: # integral gain turned off/negative setting
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
    test_mrp_steering_tracking_integrated(False, 0.15, 1.0, 0.025)
