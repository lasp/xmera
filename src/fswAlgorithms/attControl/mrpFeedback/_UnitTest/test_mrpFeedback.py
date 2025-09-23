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

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

@pytest.mark.parametrize("intGain", [0.01, -1])
@pytest.mark.parametrize("rwNum", [4, 0])
@pytest.mark.parametrize("integralLimit", [0, 20])
@pytest.mark.parametrize("ctrlLaw", [0, 1])
@pytest.mark.parametrize("useRwAvailability", ["NO", "ON", "OFF"])

def test_MRP_Feedback(show_plots, intGain, rwNum, integralLimit, ctrlLaw, useRwAvailability):
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    #   Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    #   Construct algorithm and associated C++ container
    module = mrpFeedback.MrpFeedback()
    module.modelTag = "mrpFeedback"

    #   Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    #   Initialize the test module configuration data
    module.setK(0.15)
    module.setKi(intGain)
    module.setP(150.0)
    module.setIntegralLimit(integralLimit)
    module.setControlLawType(ctrlLaw)
    module.setKnownTorquePntB_B([1., 1., 1.])

    # create input messages
    #   AttGuidFswMsg Message:
    guidCmdData = messaging.AttGuidMsgPayload()
    sigma_BR = [0.3, -0.5, 0.7]
    guidCmdData.sigma_BR = sigma_BR
    omega_BR_B = [0.010, -0.020, 0.015]
    guidCmdData.omega_BR_B = omega_BR_B
    omega_RN_B = [-0.02, -0.01, 0.005]
    guidCmdData.omega_RN_B = omega_RN_B
    domega_RN_B = [0.0002, 0.0003, 0.0001]
    guidCmdData.domega_RN_B = domega_RN_B
    guidInMsg = messaging.AttGuidMsg().write(guidCmdData)

    # vehicleConfigData Message:
    vehicleConfig = messaging.VehicleConfigMsgPayload()
    I = [1000., 0., 0.,
         0., 800., 0.,
         0., 0., 800.]
    vehicleConfig.ISCPntB_B = I
    vcInMsg = messaging.VehicleConfigMsg().write(vehicleConfig)

    # wheelSpeeds Message
    rwSpeedMessage = messaging.RWSpeedMsgPayload()
    Omega = [10.0, 25.0, 50.0, 100.0]  # rad/sec
    rwSpeedMessage.wheelSpeeds = Omega
    rwSpeedInMsg = messaging.RWSpeedMsg().write(rwSpeedMessage)

    # wheelConfigData message
    jsList = []
    GsMatrix_B = []
    if rwNum > 0:
        rwConfigParams = messaging.RWArrayConfigMsgPayload()

        GsMatrix_B = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.577350269190, 0.577350269190, 0.577350269190
        ]
        jsList = [0.1, 0.1, 0.1, 0.1]
        rwConfigParams.GsMatrix_B = GsMatrix_B
        rwConfigParams.JsList = jsList
        rwConfigParams.numRW = rwNum
        rwParamInMsg = messaging.RWArrayConfigMsg().write(rwConfigParams)

    # wheelAvailability message
    rwAvailabilityMessage = messaging.RWAvailabilityMsgPayload()
    if useRwAvailability != "NO":
        if useRwAvailability == "ON":
            rwAvailabilityMessage.wheelAvailability = [messaging.AVAILABLE, messaging.AVAILABLE,
                                                       messaging.AVAILABLE, messaging.AVAILABLE]
        elif useRwAvailability == "OFF":
            rwAvailabilityMessage.wheelAvailability = [messaging.UNAVAILABLE, messaging.UNAVAILABLE,
                                                       messaging.UNAVAILABLE, messaging.UNAVAILABLE]
        else:
            print("WARNING: unknown rw availability status")
        rwAvailInMsg = messaging.RWAvailabilityMsg().write(rwAvailabilityMessage)
    else:
        # set default availability
        rwAvailabilityMessage.wheelAvailability = [messaging.AVAILABLE, messaging.AVAILABLE,
                                                   messaging.AVAILABLE, messaging.AVAILABLE]

    LrTrue = findTrueTorques(module, guidCmdData, rwSpeedMessage, vehicleConfig, jsList,
                             rwNum, GsMatrix_B, rwAvailabilityMessage, ctrlLaw)

    #   Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.cmdTorqueOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # connect messages
    module.guidInMsg.subscribeTo(guidInMsg)
    module.vehConfigInMsg.subscribeTo(vcInMsg)
    if rwNum > 0:
        module.rwParamsInMsg.subscribeTo(rwParamInMsg)
        module.rwSpeedsInMsg.subscribeTo(rwSpeedInMsg)
    if useRwAvailability != "NO":
        module.rwAvailInMsg.subscribeTo(rwAvailInMsg)

    #   Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    #   Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    module.reset(1)     # this module reset function needs a time input (in NanoSeconds)

    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    Lr = dataLog.torqueRequestBody

    # compare the module results to the truth values
    accuracy = 1e-8
    np.testing.assert_allclose(Lr, LrTrue, atol=accuracy, rtol=0, verbose=True)


def findTrueTorques(module,guidCmdData,rwSpeedMessage,vehicleConfigOut,jsList,numRW,GsMatrix_B,rwAvailMsg,ctrlLaw):
    Lr = []

    #Read in variables
    L = np.asarray(module.getKnownTorquePntB_B()).flatten()
    steps = [0, 0, .5, 0, .5]
    omega_BR_B = np.asarray(guidCmdData.omega_BR_B)
    omega_RN_B = np.asarray(guidCmdData.omega_RN_B)
    omega_BN_B = omega_BR_B + omega_RN_B #find body rate
    domega_RN_B = np.asarray(guidCmdData.domega_RN_B)
    sigma_BR = np.asarray(guidCmdData.sigma_BR)
    Isc = np.asarray(vehicleConfigOut.ISCPntB_B)
    Isc = np.reshape(Isc, (3, 3))
    Ki = module.getKi()
    K = module.getK()
    P = module.getP()
    jsVec = jsList
    GsMatrix_B_array = np.asarray(GsMatrix_B)
    GsMatrix_B_array = np.reshape(GsMatrix_B_array[0:numRW * 3], (numRW, 3))
    sigmaInt = np.asarray([0, 0, 0])

    #Compute toruqes
    for i in range(len(steps)):
        dt = steps[i]
        if dt == 0:
            sigmaInt = np.asarray([0, 0, 0])

        #evaluate integral term
        if Ki > 0: #if integral feedback is on
            sigmaInt = K * dt * sigma_BR + sigmaInt
            for n in range(3):
                if abs(sigmaInt[n]) > module.getIntegralLimit():
                    sigmaInt[n] *= module.getIntegralLimit()/sigmaInt[n] #check elementwise if integral term is greater than limit; preserve direction (+/-)

            zVec = sigmaInt + Isc.dot(omega_BR_B)
        else: #integral gain turned off/negative setting
            zVec = np.asarray([0, 0, 0])

        #compute torque Lr
        Lr0 = K * sigma_BR  # +K*sigmaBR
        Lr1 = Lr0 + P * omega_BR_B  # +P*deltaOmega
        Lr2 = Lr1 + P * Ki * zVec  # +P*Ki*z
        GsHs = np.array([0,0,0])

        if numRW>0:
            for i in range(numRW):
                if rwAvailMsg.wheelAvailability[i] == 0:  #Make RW availability check
                    GsHs = GsHs + np.dot(GsMatrix_B_array[i, :], jsVec[i]*(np.dot(omega_BN_B, GsMatrix_B_array[i, :])+rwSpeedMessage.wheelSpeeds[i]))
                    #J_s*(dot(omegaBN_B,Gs_vec)+Omega_wheel)

        if ctrlLaw == 0:
            Lr3 = Lr2 - np.cross((omega_RN_B+Ki*zVec), (Isc.dot(omega_BN_B)+GsHs)) # -[v3Tilde(omega_r+Ki*z)]([I]omega + [Gs]h_s)
        else:
            Lr3 = Lr2 - np.cross(omega_BN_B, (Isc.dot(omega_BN_B)+GsHs)) # -[v3Tilde(omega)]([I]omega + [Gs]h_s)

        Lr4 = Lr3 + Isc.dot(-domega_RN_B + np.cross(omega_BN_B, omega_RN_B)) #+[I](-d(omega_r)/dt + omega x omega_r)
        Lr5 = Lr4 + L
        Lr5 = -Lr5
        Lr.append(np.ndarray.tolist(Lr5))
    return np.array(Lr)


if __name__ == "__main__":
    test_MRP_Feedback(False,    # showplots
                      0.01,     # intGain
                      0,        # rwNum
                      0.0,      # integralLimit
                      1,            # ctrlLaw
                      "NO"      # useRwAvailability ("NO", "ON", "OFF")
                      )
