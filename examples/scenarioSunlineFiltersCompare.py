
import numpy as np
import time

from Basilisk import __path__
bskPath = __path__[0]

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
import matplotlib.pyplot as plt
from Basilisk.utilities import orbitalMotion as om
from Basilisk.utilities import RigidBodyKinematics as rbk

from Basilisk.simulation import spacecraft, coarseSunSensor
from Basilisk.fswAlgorithms import sunlineUKF, sunlineSRuKF
from Basilisk.architecture import messaging

import SunLineKF_test_utilities as Fplot


r"""
Setup 1 - ukF
-------------

In the first run, we use an unscented Kalman Filter (:ref:`sunlineUKF`).
This filter has the following states:

================  =============
States            notation
================  =============
Sunheading        ``d``
Sunheading Rate|  ``d_dot``
================  =============


This filter estimates sunheading, and the sunheading's rate of change.
As a unscented filter, it also has the the following parameters:

=============  =============
  Name         Value
=============  =============
  ``alpha``     0.02
  ``beta``      2
  ``kappa``     0
=============  =============

The covariance is then set, as well as the measurement noise:

=============================================  ==================
  Parameter                                         Value
=============================================  ==================
  covariance on  heading vector  components         0.2
  covariance on heading rate  components            0.02
  noise on heading measurements                     0.017 ** 2
  noise on heading measurements                     0.0017 ** 2
=============================================  ==================

Result is reported below:

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
    q_noise_in = np.identity(6)
    q_noise_in[0:3, 0:3] = q_noise_in[0:3, 0:3]*0.017*0.017
    q_noise_in[3:6, 3:6] = q_noise_in[3:6, 3:6]*0.0017*0.0017
    filterObject.qNoise = q_noise_in.reshape(36).tolist()
    filterObject.qObsVal = 0.017**2
    filterObject.sensorUseThresh = np.sqrt(filterObject.qObsVal)*5


def setup_srukf_data(filter_object):
    filter_object.setAlpha(0.02)
    filter_object.setBeta(2.0)

    filter_object.setInitialPosition([1.0, 0.1, 0.0])
    filter_object.setInitialVelocity([0.0, 0.01, 0.0])
    filter_object.setInitialCovariance([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.02, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.02, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.02]])
    sigmaSun = 0.017*0.017
    sigmaRate = 0.0017*0.0017
    filter_object.setProcessNoise([[sigmaSun, 0.0, 0.0, 0.0, 0.0, 0.0],
                                   [0.0, sigmaSun, 0.0, 0.0, 0.0, 0.0],
                                   [0.0, 0.0, sigmaSun, 0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0, sigmaRate, 0.0, 0.0],
                                   [0.0, 0.0, 0.0, 0.0, sigmaRate, 0.0],
                                   [0.0, 0.0, 0.0, 0.0, 0.0, sigmaRate]])
    filter_object.cssMeasNoiseStd = 0.017
    filter_object.setCssMeasurementNoiseStd(filter_object.cssMeasNoiseStd)
    filter_object.setSensorThreshold((filter_object.cssMeasNoiseStd)*5)


def run(saveFigures, show_plots, FilterType, simTime):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        saveFigures (bool): flag to save off the figures
        show_plots (bool): Determines if the script should display plots
        FilterType (str): {'uKF', 'EKF', 'OEKF', 'SEKF', 'SuKF'}
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
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

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
    bVecLogger = None

    if FilterType == 'uKF':
        module = sunlineUKF.SunlineUKF()
        module.modelTag = "SunlineUKF"
        setup_ukf_data(module)
        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, module)
        simpleNavMsg = messaging.NavAttMsg()
        navLog = module.navStateOutMsg.recorder()
        filtLog = module.filtDataOutMsg.recorder()
        css_measurement_noise_std = np.sqrt(module.qObsVal)

    if FilterType == 'SRuKF':
        module = sunlineSRuKF.SunlineSRuKF()
        module.modelTag = "SunlineSRuKF"
        setup_srukf_data(module)
        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, module)
        simpleNavMsg = messaging.NavAttMsg()
        module.navAttInMsg.subscribeTo(simpleNavMsg)
        navLog = module.navAttOutMsg.recorder()
        filtLog = module.filterOutMsg.recorder()
        css_residual_data_log = module.filterCssResOutMsg.recorder()
        scSim.AddModelToTask(simTaskName, css_residual_data_log)
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
    start_time = time.time()
    scSim.ExecuteSimulation()
    end_time = time.time() - start_time

    #
    #   retrieve the logged data
    #
    def addTimeColumn(time, data):
        return np.transpose(np.vstack([[time], np.transpose(data)]))

    # Get messages that will make true data
    timeAxis = dataLog.times()
    OutSunPos = addTimeColumn(timeAxis, sunLog.PositionVector)
    Outr_BN_N = addTimeColumn(timeAxis, dataLog.r_BN_N)
    OutSigma_BN = addTimeColumn(timeAxis, dataLog.sigma_BN)
    Outomega_BN = addTimeColumn(timeAxis, dataLog.omega_BN_B)

    # Get the filter outputs through the messages
    stateLog = addTimeColumn(timeAxis, filtLog.state[:, range(numStates)])
    if FilterType == 'uKF':
        postFitLog = addTimeColumn(timeAxis, filtLog.postFitRes[:, :8])
    else:
        postFitLog = addTimeColumn(css_residual_data_log.times(), css_residual_data_log.postFits[:, :8])
    covarLog = addTimeColumn(timeAxis, filtLog.covar[:, range(numStates*numStates)])
    # obsLog = addTimeColumn(timeAxis, filtLog.numObs)

    sHat_B = np.zeros(np.shape(OutSunPos))
    sHatDot_B = np.zeros(np.shape(OutSunPos))
    for i in range(len(OutSunPos[:,0])):
        sHat_N = (OutSunPos[i,1:] - Outr_BN_N[i,1:])/np.linalg.norm(OutSunPos[i,1:] - Outr_BN_N[i,1:])
        dcm_BN = rbk.MRP2C(OutSigma_BN[i,1:])
        sHat_B[i,0] = sHatDot_B[i,0]= OutSunPos[i,0]
        sHat_B[i,1:] = np.dot(dcm_BN, sHat_N)
        sHatDot_B[i,1:] = - np.cross(Outomega_BN[i,1:], sHat_B[i,1:] )

    expected = np.zeros(np.shape(stateLog))
    expected[:,0:4] = sHat_B

    #   plot the results
    #
    errorVsTruth = np.copy(stateLog)
    errorVsTruth[:,1:] -= expected[:,1:]

    Fplot.StateErrorCovarPlot(errorVsTruth, covarLog, FilterType, show_plots, saveFigures)
    Fplot.StatesVsExpected(stateLog, covarLog, expected, FilterType, show_plots, saveFigures)
    Fplot.PostFitResiduals(postFitLog, css_measurement_noise_std, FilterType, show_plots, saveFigures)
    # Fplot.numMeasurements(obsLog, FilterType, show_plots, saveFigures)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")


    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return end_time

def compare_runtime(show_plots=False):
    runtime_sunline_ukf = run(False,       # save figures to file
                              show_plots,      # show_plots
                              'uKF',
                               400
                            )

    runtime_sunline_srukf = run(False,       # save figures to file
                                show_plots,      # show_plots
                                'SRuKF',
                                400
                            )
    # Header
    print(f"{'Filter':<8} | {'Runtime (s)':>12}")
    print("-" * 23)

    # Rows
    print(f"{'UKF':<8} | {runtime_sunline_ukf:12.3f}")
    print(f"{'SRuKF':<8} | {runtime_sunline_srukf:12.3f}")

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    compare_runtime(show_plots=False)
