# SPDX-License-Identifier: ISC
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import numpy as np
import time

from xmera import __path__
bskPath = __path__[0]

from xmera.utilities import SimulationBaseClass, unitTestSupport, macros
import matplotlib.pyplot as plt
from xmera.utilities import orbitalMotion as om
from xmera.utilities import RigidBodyKinematics as rbk

from xmera.simulation import spacecraft, coarseSunSensor
from xmera.fswAlgorithms import sunlineUKF, sunlineSRuKF
from xmera.architecture import messaging

import SunLineKF_test_utilities as Fplot


r"""
This scenario is adapted from scenarioCSSFilters.py to compare the runtime of two filters:
1. (baseline) sunlineUKF, and 2. (new) sunlineSRuKF.
See _Documents within for more details of each filter.

Runtime results are reported below:

Filter   |  Runtime (s)
-----------------------
UKF      |        0.050
SRuKF    |        0.164
"""

def setup_ukf_data(filterObject):
    filterObject.alpha = 0.02
    filterObject.beta = 2.0
    filterObject.kappa = 0.0

    filterObject.state = [1.0, 0.1, 0.0, 0.0, 0.01, 0.0]
    filterObject.covar = [1., 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 1., 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 1., 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.02, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.02, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.02]
    qNoise = np.identity(6)
    qNoise[0:3, 0:3] = qNoise[0:3, 0:3]*0.017*0.017
    qNoise[3:6, 3:6] = qNoise[3:6, 3:6]*0.0017*0.0017
    filterObject.qNoise = qNoise.reshape(36).tolist()
    filterObject.qObsVal = 0.017**2
    filterObject.sensorUseThresh = np.sqrt(filterObject.qObsVal)*5


def setup_srukf_data(filterObject):
    filterObject.setAlpha(0.02)
    filterObject.setBeta(2.0)

    filterObject.setInitialPosition([1.0, 0.1, 0.0])
    filterObject.setInitialVelocity([0.0, 0.01, 0.0])
    filterObject.setInitialCovariance([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                       [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                                       [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                                       [0.0, 0.0, 0.0, 0.02, 0.0, 0.0],
                                       [0.0, 0.0, 0.0, 0.0, 0.02, 0.0],
                                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.02]])
    sigmaSun = 0.017*0.017
    sigmaRate = 0.0017*0.0017
    filterObject.setProcessNoise([[sigmaSun, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  [0.0, sigmaSun, 0.0, 0.0, 0.0, 0.0],
                                  [0.0, 0.0, sigmaSun, 0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, sigmaRate, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, sigmaRate, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, 0.0, sigmaRate]])
    filterObject.cssMeasNoiseStd = 0.017
    filterObject.setCssMeasurementNoiseStd(filterObject.cssMeasNoiseStd)
    filterObject.setSensorThreshold((filterObject.cssMeasNoiseStd) * 5)


def run(saveFigures, showPlots, filterType, simTime):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        saveFigures (bool): flag to save off the figures
        showPlots (bool): Determines if the script should display plots
        filterType (str): {'uKF', 'SRuKF'}
        simTime (float): The length of the simulation time

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.sec2nano(simTime)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(0.5)
    dynProcess.addTask(simTaskName, simulationTimeStep)

    #
    #   setup the simulation tasks/objects
    #
    # create sun position message at origin
    sunMsgData = messaging.SpicePlanetStateMsgPayload()
    sunMsg = messaging.SpicePlanetStateMsg().write(sunMsgData)
    sunLog = sunMsg.recorder()
    scSim.AddModelToTask(simTaskName, sunLog)

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.modelTag = "bsk-Sat"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0                   # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]] # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    #
    # set initial spacecraft states
    #
    scObject.hub.r_CN_NInit = [[-om.AU*1000.], [0.0], [0.0]]              # m   - r_CN_N
    scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]                 # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.]]               # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[-0.1*macros.D2R], [0.5*macros.D2R], [0.5*macros.D2R]]   # rad/s - omega_BN_B

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    dataLog = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, dataLog)


    # Make a CSS constelation
    cssConstelation = coarseSunSensor.CSSConstellation()
    CSSOrientationList = [
        [0.70710678118654746, -0.5, 0.5],
        [0.70710678118654746, -0.5, -0.5],
        [0.70710678118654746, 0.5, -0.5],
        [0.70710678118654746, 0.5, 0.5],
        [-0.70710678118654746, 0, 0.70710678118654757],
        [-0.70710678118654746, 0.70710678118654757, 0.0],
        [-0.70710678118654746, 0, -0.70710678118654757],
        [-0.70710678118654746, -0.70710678118654757, 0.0]
    ]
    counter = 0
    def setupCSS(CSS):
        CSS.minOutput = 0.
        CSS.senNoiseStd = 0.017
        CSS.sunInMsg.subscribeTo(sunMsg)
        CSS.stateInMsg.subscribeTo(scObject.scStateOutMsg)
        CSS.this.disown()
    for CSSHat in CSSOrientationList:
        newCSS = coarseSunSensor.CoarseSunSensor()
        newCSS.modelTag = "CSS" + str(counter)
        counter += 1
        setupCSS(newCSS)
        newCSS.nHat_B = CSSHat
        cssConstelation.appendCSS(newCSS)
    scSim.AddModelToTask(simTaskName, cssConstelation)

    #
    #   add the FSW CSS information
    #
    cssConstVehicle = messaging.CSSConfigMsgPayload()

    totalCSSList = []
    for CSSHat in CSSOrientationList:
        newCSS = messaging.CSSUnitConfigMsgPayload()
        newCSS.nHat_B = CSSHat
        newCSS.CBias = 1.0
        totalCSSList.append(newCSS)
    cssConstVehicle.nCSS = len(CSSOrientationList)
    cssConstVehicle.cssVals = totalCSSList

    cssConstMsg = messaging.CSSConfigMsg().write(cssConstVehicle)

    #
    # Setup filter
    #
    numStates = 6

    if filterType == 'uKF':
        module = sunlineUKF.SunlineUKF()
        module.modelTag = "SunlineUKF"
        setup_ukf_data(module)
        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, module)
        simpleNavMsg = messaging.NavAttMsg()
        navLog = module.navStateOutMsg.recorder()
        filtLog = module.filtDataOutMsg.recorder()
        css_measurement_noise_std = np.sqrt(module.qObsVal)

    if filterType == 'SRuKF':
        module = sunlineSRuKF.SunlineSRuKF()
        module.modelTag = "SunlineSRuKF"
        setup_srukf_data(module)
        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, module)
        simpleNavMsg = messaging.NavAttMsg()
        module.navAttInMsg.subscribeTo(simpleNavMsg)
        navLog = module.navAttOutMsg.recorder()
        filtLog = module.filterOutMsg.recorder()
        cssResidualLog = module.filterCssResOutMsg.recorder()
        scSim.AddModelToTask(simTaskName, cssResidualLog)
        css_measurement_noise_std = module.getCssMeasurementNoiseStd()

    module.cssDataInMsg.subscribeTo(cssConstelation.constellationOutMsg)
    module.cssConfigInMsg.subscribeTo(cssConstMsg)

    scSim.AddModelToTask(simTaskName, navLog)
    scSim.AddModelToTask(simTaskName, filtLog)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)

    # Time the runs for performance comparisons
    startTime = time.time()
    scSim.ExecuteSimulation()
    endTime = time.time() - startTime

    #
    #   retrieve the logged data
    #
    def addTimeColumn(time, data):
        return np.transpose(np.vstack([[time], np.transpose(data)]))

    # Get messages that will make true data
    timeAxis = dataLog.times()
    outSunPos = addTimeColumn(timeAxis, sunLog.PositionVector)
    outR_BN_N = addTimeColumn(timeAxis, dataLog.r_BN_N)
    outSigma_BN = addTimeColumn(timeAxis, dataLog.sigma_BN)
    outOmega_BN = addTimeColumn(timeAxis, dataLog.omega_BN_B)

    # Get the filter outputs through the messages
    stateLog = addTimeColumn(timeAxis, filtLog.state[:, range(numStates)])
    if filterType == 'uKF':
        postFitLog = addTimeColumn(timeAxis, filtLog.postFitRes[:, :8])
    else:
        postFitLog = addTimeColumn(cssResidualLog.times(), cssResidualLog.postFits[:, :8])
    covarLog = addTimeColumn(timeAxis, filtLog.covar[:, range(numStates**2)])

    sHat_B = np.zeros(np.shape(outSunPos))
    sHatDot_B = np.zeros(np.shape(outSunPos))
    for i in range(len(outSunPos[:,0])):
        sHat_N = (outSunPos[i,1:] - outR_BN_N[i, 1:]) / np.linalg.norm(outSunPos[i, 1:] - outR_BN_N[i, 1:])
        dcm_BN = rbk.MRP2C(outSigma_BN[i, 1:])
        sHat_B[i,0] = sHatDot_B[i,0]= outSunPos[i,0]
        sHat_B[i,1:] = np.dot(dcm_BN, sHat_N)
        sHatDot_B[i,1:] = - np.cross(outOmega_BN[i,1:], sHat_B[i,1:] )

    expected = np.zeros(np.shape(stateLog))
    expected[:,0:4] = sHat_B

    #   plot the results
    errorVsTruth = np.copy(stateLog)
    errorVsTruth[:,1:] -= expected[:,1:]

    Fplot.StateErrorCovarPlot(errorVsTruth, covarLog, filterType, showPlots, saveFigures)
    Fplot.StatesVsExpected(stateLog, covarLog, expected, filterType, showPlots, saveFigures)
    Fplot.PostFitResiduals(postFitLog, css_measurement_noise_std, filterType, showPlots, saveFigures)

    if showPlots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return endTime

def compare_runtime(showPlots=False):
    runtimeSunlineUkf = run(False,
                            showPlots,
                              'uKF',
                            400
                            )

    runtimeSunlineSRukf = run(False,
                              showPlots,
                                'SRuKF',
                              400
                              )
    # Header
    print(f"{'Filter':<8} | {'Runtime (s)':>12}")
    print("-" * 23)

    # Rows
    print(f"{'UKF':<8} | {runtimeSunlineUkf:12.3f}")
    print(f"{'SRuKF':<8} | {runtimeSunlineSRukf:12.3f}")

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    compare_runtime(showPlots=True)
