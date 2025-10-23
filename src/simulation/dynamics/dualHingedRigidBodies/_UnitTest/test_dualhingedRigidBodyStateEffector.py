
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


import inspect
import os

import matplotlib.pyplot as plt
import numpy
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from xmera.utilities import SimulationBaseClass
from xmera.utilities import unitTestSupport
from xmera.simulation import spacecraft
from xmera.simulation import dualHingedRigidBodyStateEffector
from xmera.simulation import gravityEffector
from xmera.utilities import macros
from xmera.architecture import messaging

@pytest.mark.parametrize("useFlag, testCase", [
    (False, 'NoGravity'),
    (False, 'Gravity')
])
def test_dualHingedRigidBody(show_plots, useFlag, testCase):
    scObject = spacecraft.Spacecraft()
    scObject.modelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.0001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    unitTestSim.panel1 = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    unitTestSim.panel2 = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unitTestSim.panel1.modelTag = "panel1"
    unitTestSim.panel1.mass1 = 50.0
    unitTestSim.panel1.IPntS1_S1 = [[50.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]]
    unitTestSim.panel1.d1 = 0.75
    unitTestSim.panel1.l1 = 1.5
    unitTestSim.panel1.k1 = 100.0
    unitTestSim.panel1.c1 = 0.0
    unitTestSim.panel1.r_H1B_B = [[0.5], [0.0], [1.0]]
    unitTestSim.panel1.dcm_H1B = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel1.mass2 = 50.0
    unitTestSim.panel1.IPntS2_S2 = [[50.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]]
    unitTestSim.panel1.d2 = 0.75
    unitTestSim.panel1.k2 = 100.0
    unitTestSim.panel1.c2 = 0.0
    unitTestSim.panel1.theta1Init = 5*numpy.pi/180.0
    unitTestSim.panel1.theta1DotInit = 0.0
    unitTestSim.panel1.theta2Init = 0.0
    unitTestSim.panel1.theta2DotInit = 0.0

    # Define Variables for panel 2
    unitTestSim.panel2.modelTag = "panel2"
    unitTestSim.panel2.mass1 = 50.0
    unitTestSim.panel2.IPntS1_S1 = [[50.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]]
    unitTestSim.panel2.d1 = 0.75
    unitTestSim.panel2.l1 = 1.5
    unitTestSim.panel2.k1 = 100.0
    unitTestSim.panel2.c1 = 0.0
    unitTestSim.panel2.r_H1B_B = [[-0.5], [0.0], [1.0]]
    unitTestSim.panel2.dcm_H1B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel2.mass2 = 50.0
    unitTestSim.panel2.IPntS2_S2 = [[50.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]]
    unitTestSim.panel2.d2 = 0.75
    unitTestSim.panel2.k2 = 100.0
    unitTestSim.panel2.c2 = 0.0
    unitTestSim.panel2.theta1Init = 5*numpy.pi/180.0
    unitTestSim.panel2.theta1DotInit = 0.0
    unitTestSim.panel2.theta2Init = 0.0
    unitTestSim.panel2.theta2DotInit = 0.0

    # Add panels to spaceCraft
    # this next line is not working
    scObject.addStateEffector(unitTestSim.panel1)
    scObject.addStateEffector(unitTestSim.panel2)

    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[0.1], [-0.4], [0.3]]
    scObject.hub.v_CN_NInit = [[-0.2], [0.5], [0.1]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    if testCase == 'Gravity':
        unitTestSim.earthGravBody = gravityEffector.GravBodyData()
        unitTestSim.earthGravBody.planetName = "earth_planet_data"
        unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
        unitTestSim.earthGravBody.isCentralBody = True
        scObject.gravField.gravBodies = spacecraft.GravBodyVector([unitTestSim.earthGravBody])
        scObject.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
        scObject.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]

    dataLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Add energy and momentum s to log
    scObjectLog = scObject.logger(["totOrbEnergy", "totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    unitTestSim.InitializeSimulation()

    stopTime = 1.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    orbEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbEnergy)
    orbAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbAngMomPntN_N)
    rotAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotAngMomPntC_N)
    rotEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotEnergy)

    initialOrbAngMom_N = [
                [orbAngMom_N[0, 1], orbAngMom_N[0, 2], orbAngMom_N[0, 3]]
                ]

    finalOrbAngMom = [
                [orbAngMom_N[-1, 1], orbAngMom_N[-1, 2], orbAngMom_N[-1, 3]]
                 ]

    initialRotAngMom_N = [
                [rotAngMom_N[0, 1], rotAngMom_N[0, 2], rotAngMom_N[0, 3]]
                ]

    finalRotAngMom = [
                [rotAngMom_N[-1, 1], rotAngMom_N[-1, 2], rotAngMom_N[-1, 3]]
                 ]

    initialOrbEnergy = [
                [orbEnergy[0, 1]]
                ]

    finalOrbEnergy = [
                [orbEnergy[-1, 1]]
                 ]

    initialRotEnergy = [
                [rotEnergy[int(len(rotEnergy)/2)+1, 1]]
                ]

    finalRotEnergy = [
                [rotEnergy[-1, 1]]
                 ]

    plt.close('all')
    plt.figure()
    plt.clf()
    plt.plot(orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("ChangeInOrbitalAngularMomentum" + testCase, "Change in Orbital Angular Momentum " + testCase, plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("ChangeInOrbitalEnergy" + testCase, "Change in Orbital Energy " + testCase, plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("ChangeInRotationalAngularMomentum" + testCase, "Change in Rotational Angular Momentum " + testCase, plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(rotEnergy[:,0]*1e-9, (rotEnergy[:,1] - rotEnergy[0,1])/rotEnergy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("ChangeInRotationalEnergy" + testCase, "Change in Rotational Energy " + testCase, plt, r"width=0.8\textwidth", path)
    if show_plots:
        plt.show()
        plt.close("all")

    accuracy = 1e-10
    numpy.testing.assert_allclose(numpy.array(initialOrbAngMom_N),
                                  numpy.array(finalOrbAngMom),
                                  rtol=accuracy,
                                  err_msg="Dual Hinged Rigid Body Integrated Test failed orbital angular momentum unit "
                                       "test")

    numpy.testing.assert_allclose(numpy.array(initialRotAngMom_N),
                                  numpy.array(finalRotAngMom),
                                  rtol=accuracy,
                                  err_msg="Dual Hinged Rigid Body Integrated Test failed rotational angular momentum "
                                          "unit test")

    numpy.testing.assert_allclose(numpy.array(initialOrbEnergy),
                                  numpy.array(finalOrbEnergy),
                                  rtol=accuracy,
                                  err_msg="Dual Hinged Rigid Body Integrated Test failed orbital energy unit test")

    numpy.testing.assert_allclose(numpy.array(initialRotEnergy),
                                  numpy.array(finalRotEnergy),
                                  rtol=accuracy,
                                  err_msg="Dual Hinged Rigid Body Integrated Test failed rotational energy unit test")


def test_dualHingedRigidBodyMotorTorque(show_plots):
    scObject = spacecraft.Spacecraft()
    scObject.modelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.01)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    unitTestSim.panel1 = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    unitTestSim.panel2 = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unitTestSim.panel1.modelTag = "panel1"
    unitTestSim.panel1.mass1 = 50.0
    unitTestSim.panel1.IPntS1_S1 = [[50.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]]
    unitTestSim.panel1.d1 = 0.75
    unitTestSim.panel1.l1 = 1.5
    unitTestSim.panel1.k1 = 0.0
    unitTestSim.panel1.c1 = 0.0
    unitTestSim.panel1.r_H1B_B = [[0.5], [0.0], [1.0]]
    unitTestSim.panel1.dcm_H1B = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel1.mass2 = 50.0
    unitTestSim.panel1.IPntS2_S2 = [[50.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]]
    unitTestSim.panel1.d2 = 0.75
    unitTestSim.panel1.k2 = 100.0
    unitTestSim.panel1.c2 = 0.0
    unitTestSim.panel1.theta1Init = 0*numpy.pi/180.0
    unitTestSim.panel1.theta1DotInit = 0.0
    unitTestSim.panel1.theta2Init = 0.0
    unitTestSim.panel1.theta2DotInit = 0.0

    # set a fixed motor torque message
    motorMsgData = messaging.ArrayMotorTorqueMsgPayload()
    motorMsgData.motorTorque = [2.0, 4.0]
    motorMsg = messaging.ArrayMotorTorqueMsg().write(motorMsgData)
    unitTestSim.panel1.motorTorqueInMsg.subscribeTo(motorMsg)

    # Define Variables for panel 2
    unitTestSim.panel2.modelTag = "panel2"
    unitTestSim.panel2.mass1 = 50.0
    unitTestSim.panel2.IPntS1_S1 = [[50.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]]
    unitTestSim.panel2.d1 = 0.75
    unitTestSim.panel2.l1 = 1.5
    unitTestSim.panel2.k1 = 0.0
    unitTestSim.panel2.c1 = 0.0
    unitTestSim.panel2.r_H1B_B = [[-0.5], [0.0], [1.0]]
    unitTestSim.panel2.dcm_H1B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel2.nameOfTheta1State = "dualHingedRigidBody2Theta1"
    unitTestSim.panel2.nameOfTheta1DotState = "dualHingedRigidBody2ThetaDot1"
    unitTestSim.panel2.mass2 = 50.0
    unitTestSim.panel2.IPntS2_S2 = [[50.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]]
    unitTestSim.panel2.d2 = 0.75
    unitTestSim.panel2.k2 = 0.0
    unitTestSim.panel2.c2 = 0.0
    unitTestSim.panel2.nameOfTheta2State = "dualHingedRigidBody2Theta2"
    unitTestSim.panel2.nameOfTheta2DotState = "dualHingedRigidBody2ThetaDot2"
    unitTestSim.panel2.theta1Init = 0 * numpy.pi / 180.0
    unitTestSim.panel2.theta1DotInit = 0.0
    unitTestSim.panel2.theta2Init = 0.0
    unitTestSim.panel2.theta2DotInit = 0.0

    # Add panels to spaceCraft
    scObject.addStateEffector(unitTestSim.panel1)
    scObject.addStateEffector(unitTestSim.panel2)

    # Define mass properties of the rigid part of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)
    unitTestSim.AddModelToTask(unitTaskName, unitTestSim.panel1)
    unitTestSim.AddModelToTask(unitTaskName, unitTestSim.panel2)

    dataLog = scObject.scStateOutMsg.recorder()
    dataPanel10Log = unitTestSim.panel1.dualHingedRigidBodyOutMsgs[0].recorder()
    dataPanel11Log = unitTestSim.panel1.dualHingedRigidBodyOutMsgs[1].recorder()
    dataPanel20Log = unitTestSim.panel2.dualHingedRigidBodyOutMsgs[0].recorder()
    dataPanel21Log = unitTestSim.panel2.dualHingedRigidBodyOutMsgs[1].recorder()
    data10Log = unitTestSim.panel1.dualHingedRigidBodyConfigLogOutMsgs[0].recorder()
    data21Log = unitTestSim.panel2.dualHingedRigidBodyConfigLogOutMsgs[1].recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)
    unitTestSim.AddModelToTask(unitTaskName, dataPanel10Log)
    unitTestSim.AddModelToTask(unitTaskName, dataPanel11Log)
    unitTestSim.AddModelToTask(unitTaskName, dataPanel20Log)
    unitTestSim.AddModelToTask(unitTaskName, dataPanel21Log)
    unitTestSim.AddModelToTask(unitTaskName, data10Log)
    unitTestSim.AddModelToTask(unitTaskName, data21Log)

    scLog = scObject.logger("totRotAngMomPntC_N")
    unitTestSim.AddModelToTask(unitTaskName, scLog)

    unitTestSim.InitializeSimulation()

    stopTime = 10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    rOut_CN_N = dataLog.r_CN_N
    vOut_CN_N = dataLog.v_CN_N
    sigma_BN = dataLog.sigma_BN
    thetaP1A1 = dataPanel10Log.theta
    thetaP1A2 = dataPanel11Log.theta
    thetaP2A1 = dataPanel20Log.theta
    thetaP2A2 = dataPanel21Log.theta
    rB1N = data10Log.r_BN_N[0]
    vB1N = data10Log.v_BN_N[0]
    sB1N = data10Log.sigma_BN[0]
    oB1N = data10Log.omega_BN_B[0]
    rB2N = data21Log.r_BN_N[0]
    vB2N = data21Log.v_BN_N[0]
    sB2N = data21Log.sigma_BN[0]
    oB2N = data21Log.omega_BN_B[0]

    rotAngMom_N = unitTestSupport.addTimeColumn(scLog.times(), scLog.totRotAngMomPntC_N)

    # Get the last sigma and position
    dataPos = [rOut_CN_N[-1]]

    truePos = [[0., 0., 0.]]

    initialRotAngMom_N = [[rotAngMom_N[0, 1], rotAngMom_N[0, 2], rotAngMom_N[0, 3]]]

    finalRotAngMom = [rotAngMom_N[-1, 1:4]]

    plt.close("all")

    plt.figure()
    plt.clf()
    plt.plot(rotAngMom_N[:, 0] * 1e-9, (rotAngMom_N[:, 1] - rotAngMom_N[0, 1]) ,
             rotAngMom_N[:, 0] * 1e-9, (rotAngMom_N[:, 2] - rotAngMom_N[0, 2]) ,
             rotAngMom_N[:, 0] * 1e-9, (rotAngMom_N[:, 3] - rotAngMom_N[0, 3]) )
    plt.xlabel('time (s)')
    plt.ylabel('Ang. Momentum Difference')

    plt.figure()
    plt.clf()
    plt.plot(dataLog.times() * 1e-9, vOut_CN_N[:, 0], dataLog.times() * 1e-9, vOut_CN_N[:, 1], dataLog.times() * 1e-9,
             vOut_CN_N[:, 2])
    plt.xlabel('time (s)')
    plt.ylabel('m/s')

    plt.figure()
    plt.clf()
    plt.plot(dataLog.times() * macros.NANO2SEC, sigma_BN[:, 0],
             color=unitTestSupport.getLineColor(0, 3),
             label=r'$\sigma_{1}$')
    plt.plot(dataLog.times() * macros.NANO2SEC, sigma_BN[:, 1],
             color=unitTestSupport.getLineColor(1, 3),
             label=r'$\sigma_{2}$')
    plt.plot(dataLog.times() * macros.NANO2SEC, sigma_BN[:, 2],
             color=unitTestSupport.getLineColor(2, 3),
             label=r'$\sigma_{3}$')
    plt.legend(loc='lower right')
    plt.xlabel('time (s)')
    plt.ylabel(r'MRP $\sigma_{B/N}$')

    plt.figure()
    plt.clf()
    plt.plot(dataPanel10Log.times() * macros.NANO2SEC, thetaP1A1*macros.R2D,
             color=unitTestSupport.getLineColor(0, 4),
             label=r'Panel 1 $\theta_{1}$')
    plt.plot(dataPanel10Log.times() * macros.NANO2SEC, thetaP1A2*macros.R2D,
             color=unitTestSupport.getLineColor(1, 4),
             label=r'Panel 1 $\theta_{2}$')
    plt.plot(dataPanel10Log.times() * macros.NANO2SEC, thetaP2A1 * macros.R2D,
             color=unitTestSupport.getLineColor(2, 4),
             label=r'Panel 2 $\theta_{1}$')
    plt.plot(dataPanel10Log.times() * macros.NANO2SEC, thetaP2A2 * macros.R2D,
             color=unitTestSupport.getLineColor(3, 4),
             label=r'Panel 2 $\theta_{2}$')
    plt.legend(loc='lower right')
    plt.xlabel('time (s)')
    plt.ylabel('Hinge Angles [deg]')

    if show_plots:
        plt.show()
    plt.close("all")

    accuracy = 1e-10
    numpy.testing.assert_allclose(numpy.array(dataPos),
                                  numpy.array(truePos),
                                  atol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed position test")

    numpy.testing.assert_allclose(numpy.array(finalRotAngMom),
                                  numpy.array(initialRotAngMom_N),
                                  atol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed rotational angular momentum unit test")

    # check config log messages
    numpy.testing.assert_allclose(rB1N,
                                  [1.25, 0, 0],
                                  atol=accuracy,
                                  err_msg="Dual Hinged Rigid Body integrated test failed panel 1 r_S1N_N config log "
                                          "test")

    numpy.testing.assert_allclose(vB1N,
                                  [0.0, 0, 0],
                                  atol=accuracy,
                                  err_msg="Dual Hinged Rigid Body integrated test failed panel 1 v_S1N_N config log "
                                          "test")

    numpy.testing.assert_allclose(sB1N,
                                  [0.0, 0, 1.0],
                                  atol=accuracy,
                                  err_msg="Dual Hinged Rigid Body integrated test failed panel 1 sigma_S1N config log "
                                          "test")

    numpy.testing.assert_allclose(oB1N,
                                  [0.0, 0, 0],
                                  atol=accuracy,
                                  err_msg="Dual Hinged Rigid Body integrated test failed panel 1 omega_S1N_B config log "
                                          "test")

    numpy.testing.assert_allclose(rB2N,
                                  [-2.75, 0, 0],
                                  atol=accuracy,
                                  err_msg="Dual Hinged Rigid Body integrated test failed panel 2 r_S2N_N config log "
                                          "test")

    numpy.testing.assert_allclose(vB2N,
                                  [0.0, 0, 0],
                                  atol=accuracy,
                                  err_msg="Dual Hinged Rigid Body integrated test failed panel 2 v_S2N_N config log "
                                          "test")

    numpy.testing.assert_allclose(sB2N,
                                  [0.0, 0, 0.0],
                                  atol=accuracy,
                                  err_msg="Dual Hinged Rigid Body integrated test failed panel 2 sigma_S2N config log "
                                          "test")

    numpy.testing.assert_allclose(oB2N,
                                  [0.0, 0, 0],
                                  atol=accuracy,
                                  err_msg="Dual Hinged Rigid Body integrated test failed panel 2 omega_S2N_B config "
                                          "log test")


if __name__ == "__main__":
    test_dualHingedRigidBody(True, False, 'NoGravity')
    # dualHingedRigidBodyMotorTorque(True, True)
