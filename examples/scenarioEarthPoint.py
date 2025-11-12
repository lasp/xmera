r"""
Overview
--------

This script shows how to go into earth point state. The spacecraft is equipped with two large rotating solar arrays (SAs) which can continuously track the Sun to ensure maximum power generation.
The goals for the earth point are to continuously point the body +Y axis to earth, while also keeping the X axis orthogonal to the sun and the +Z axis as far away from the sun as possible.
The main flight software modules used in this script are
the following:

- :ref:`triad`: computes the reference attitude for a spacecraft using the triad method to statisfy the requirements of the state.

The script is found in the folder ``xmera/examples`` and executed by using::

      python3 scenarioEarthPoint.py

"""

import os

import matplotlib.pyplot as plt
import numpy as np
from xmera import __path__
from xmera.architecture import messaging
from xmera.fswAlgorithms import (mrpFeedback, attTrackingError, triad, rwMotorTorque,
                                    hingedRigidBodyPIDMotor, solarArrayReference, thrusterPlatformReference,
                                    thrusterPlatformState, thrustCMEstimation, torqueScheduler)
from xmera.simulation import (reactionWheelStateEffector, simpleNav, simpleMassProps, spacecraft,
                                 spinningBodyOneDOFStateEffector,
                                 spinningBodyTwoDOFStateEffector, thrusterStateEffector, facetSRPDynamicEffector, ephemerisConverter, boreAngCalc)
from xmera.utilities import (SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody, simIncludeRW,
                                unitTestSupport, vizSupport, RigidBodyKinematics as rbk)

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

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

def angle_between_vectors(v1, v2):
    v1 = np.array(v1, dtype=float)
    v2 = np.array(v2, dtype=float)
    dot_product = np.dot(v1, v2)
    norm_product = np.linalg.norm(v1) * np.linalg.norm(v2)

    # Ensure the value is within the valid range for arccos
    cos_theta = np.clip(dot_product / norm_product, -1.0, 1.0)

    return np.arccos(cos_theta)



def run( showPlots):
    """
    The scenario can be run with the followings setups parameters:

    Args:
        showPlots (bool): Determines if the script should display plots.

    """

    # Create simulation variable names
    fswTask = "fswTask"
    dynTask = "dynTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.SetProgressBar(True)

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the simulation time and integration update time
    simulationTime = macros.hour2nano(2)
    simulationTimeStepDyn = macros.sec2nano(0.05)
    simulationTimeStepFsw = macros.sec2nano(0.5)
    dynProcess.addTask(scSim.CreateNewTask(dynTask, simulationTimeStepDyn))
    dynProcess.addTask(scSim.CreateNewTask(fswTask, simulationTimeStepFsw))


#
    # setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.modelTag = "Spacecraft"

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(dynTask, scObject, 1)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # Next a series of gravitational bodies are included
    gravBodies = gravFactory.createBodies(['sun','earth'])
    gravBodies['sun'].isCentralBody = True
    mu = gravBodies['sun'].mu

    # The configured gravitational bodies are added to the spacecraft dynamics with the usual command:
    gravFactory.addBodiesTo(scObject)

    # Next, the default SPICE support module is created and configured.
    timeInitString = "2023 OCTOBER 22 00:00:00.0"

    # The following is a support macro that creates a `gravFactory.spiceObject` instance
    gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)

    # Sun is gravity center
    gravFactory.spiceObject.zeroBase = 'Sun'

    # The SPICE object is added to the simulation task list.
    scSim.AddModelToTask(fswTask, gravFactory.spiceObject, 27)

    ephemeris = ephemerisConverter.EphemerisConverter()
    ephemeris.modelTag = "ephemData"
    ephemeris.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[0])
    ephemeris.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[1])

    scSim.AddModelToTask(fswTask, ephemeris, 26)

    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 7.48e10      # meters
    oe.e = 0.00
    oe.i = 0.0 * macros.D2R
    oe.Omega = 0.0 * macros.D2R
    oe.omega = 0.0 * macros.D2R
    oe.f = 0.0 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    # To set the spacecraft initial conditions, the following initial position and velocity variables are set:
    scObject.hub.r_CN_NInit = rN                          # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN                          # m/s - v_BN_N
    scObject.hub.sigma_BNInit = [0, 0., 0.]              # MRP set to customize initial inertial attitude
    scObject.hub.omega_BN_BInit = [[0.], [0.], [0.]]      # rad/s - omega_CN_B

    # define the simulation inertia
    I = [ 1725,    -5,   -12,
            -5,  5525,    43,
            -12,   43,  4810]
    scObject.hub.mHub = 2500  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.008], [-0.010], [1.214]]  # [m] - position vector of hub CM relative to the body-fixed point B
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # create the FSW vehicle configuration message for inertias
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I       # use the same inertia in the FSW algorithm as in the simulation
    vcMsg_I = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    #
    # add RW devices
    #
    # Make RW factory instance
    rwFactory = simIncludeRW.rwFactory()

    # specify RW momentum capacity
    maxRWMomentum = 100.  # Nms

    # Define orthogonal RW pyramid
    # -- Pointing directions
    rwElAngle = np.array([40.0, 40.0, 40.0, 40.0]) * macros.D2R
    rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0]) * macros.D2R
    rwPosVector = [[0.8, 0.8, 1.8],
                    [0.8, -0.8, 1.8],
                    [-0.8, -0.8, 1.8],
                    [-0.8, 0.8, 1.8]]

    Gs = []
    for elAngle, azAngle, posVector in zip(rwElAngle, rwAzimuthAngle, rwPosVector):
        gsHat = (rbk.Mi(-azAngle, 3).dot(rbk.Mi(elAngle, 2))).dot(np.array([1, 0, 0]))
        Gs.append(gsHat)
        rwFactory.create('Honeywell_HR16', gsHat, maxMomentum=maxRWMomentum, rWB_B=posVector, Omega=0.)

    numRW = rwFactory.getNumOfDevices()

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.modelTag = "RW_cluster"
    rwFactory.addToSpacecraft(scObject.modelTag, rwStateEffector, scObject)

    # add RW object array to the simulation process
    scSim.AddModelToTask(dynTask, rwStateEffector, 2)

    # Setup the FSW RW configuration message.
    fswRwConfigMsg = rwFactory.getConfigMessage()

    # add the simple Navigation sensor module
    sNavObject = simpleNav.SimpleNav()
    sNavObject.modelTag = "SimpleNavigation"
    scSim.AddModelToTask(dynTask, sNavObject)


    # Set up the rotating solar arrays
    numRSA = 2
    RSAList = []
    # 1st solar array
    RSAList.append(spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector())
    scSim.AddModelToTask(dynTask, RSAList[0])
    RSAList[0].r_SB_B = [0.75, 0.0, 0.45]
    RSAList[0].r_ScS_S = [0.0, 3.75, 0.0]
    RSAList[0].sHat_S = [0, 1, 0]
    RSAList[0].dcm_S0B = [[0, 0, -1], [1, 0, 0], [0, -1, 0]]
    RSAList[0].IPntSc_S = [[250.0, 0.0, 0.0],
                           [0.0, 250.0, 0.0],
                           [0.0, 0.0, 500.0]]
    RSAList[0].mass = 85
    RSAList[0].k = 0
    RSAList[0].c = 0
    RSAList[0].thetaInit = 0
    RSAList[0].thetaDotInit = 0
    RSAList[0].modelTag = "solarArray1"
    scObject.addStateEffector(RSAList[0])
    # 2nd solar array
    RSAList.append(spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector())
    scSim.AddModelToTask(dynTask, RSAList[1])
    RSAList[1].r_SB_B = [-0.75, 0.0, 0.45]
    RSAList[1].r_ScS_S = [0.0, 3.75, 0.0]
    RSAList[1].sHat_S = [0, 1, 0]
    RSAList[1].dcm_S0B = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    RSAList[1].IPntSc_S = [[250.0, 0.0, 0.0],
                           [0.0, 250.0, 0.0],
                           [0.0, 0.0, 500.0]]
    RSAList[1].mass = 85
    RSAList[1].k = 0
    RSAList[1].c = 0
    RSAList[1].thetaInit = 0
    RSAList[1].thetaDotInit = 0
    RSAList[1].modelTag = "solarArray2"
    scObject.addStateEffector(RSAList[1])

    # Boresight vector modules.
    zBACObject = boreAngCalc.BoreAngCalc()
    zBACObject.modelTag = "zBoresight"
    zBACObject.boreVec_B = [0., 0., 1.]  # boresight in body frame
    scSim.AddModelToTask(dynTask, zBACObject)

    yBACObject = boreAngCalc.BoreAngCalc()
    yBACObject.modelTag = "yBoresight"
    yBACObject.boreVec_B = [0., 1., 0.]  # boresight in body frame
    scSim.AddModelToTask(dynTask, yBACObject)

    xBACObject = boreAngCalc.BoreAngCalc()
    xBACObject.modelTag = "xBoresight"
    xBACObject.boreVec_B = [1., 0., 0.]  # boresight in body frame
    scSim.AddModelToTask(dynTask, xBACObject)

    #
    #   setup the FSW algorithm modules


    # # Set up attitude guidance module
    earthPoint = triad.Triad()
    earthPoint.modelTag = "earthPointGuidance"
    earthPoint.setA1Hat_B ([1, 0, 0])          # solar array drive axis
    earthPoint.setH1Hat_B ([0, 1, 0])          # random inertial thrust direction
    scSim.AddModelToTask(fswTask, earthPoint, 25)

    # Set up the solar array reference modules
    saReference = []
    for item in range(numRSA):
        saReference.append(solarArrayReference.SolarArrayReference())
        saReference[item].modelTag = "SolarArrayReference"+str(item+1)
        saReference[item].a1Hat_B = [(-1)**item, 0, 0]
        saReference[item].a2Hat_B = [0, 1, 0]
        scSim.AddModelToTask(fswTask, saReference[item], 24)

    # Set up solar array controller modules
    saController = []
    for item in range(numRSA):
        saController.append(hingedRigidBodyPIDMotor.HingedRigidBodyPIDMotor())
        saController[item].modelTag = "SolarArrayMotor"+str(item+1)
        saController[item].K = 1.25
        saController[item].P = 50
        saController[item].I = 3e-3
        scSim.AddModelToTask(fswTask, saController[item], 23)

    # Set up attitude tracking error
    attError = attTrackingError.AttTrackingError()
    attError.modelTag = "AttitudeTrackingError"
    scSim.AddModelToTask(fswTask, attError, 22)

    # Set up the MRP Feedback control module
    mrpControl = mrpFeedback.MrpFeedback()
    mrpControl.modelTag = "mrpFeedback"
    mrpControl.setKi(1e-5)
    mrpControl.setP(50)
    mrpControl.setK(2)
    mrpControl.setIntegralLimit(2. / mrpControl.getKi() * 0.1)
    mrpControl.setControlLawType(1)
    scSim.AddModelToTask(fswTask, mrpControl, 21)

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueObj = rwMotorTorque.RwMotorTorque()
    rwMotorTorqueObj.modelTag = "rwMotorTorque"
    rwMotorTorqueObj.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    scSim.AddModelToTask(fswTask, rwMotorTorqueObj, 20)

    # Connect messages
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    sNavObject.sunStateInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[0])
    RSAList[0].motorTorqueInMsg.subscribeTo(saController[0].motorTorqueOutMsg)
    RSAList[1].motorTorqueInMsg.subscribeTo(saController[1].motorTorqueOutMsg)
    earthPoint.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    earthPoint.ephemerisInMsg.subscribeTo(ephemeris.ephemOutMsgs[1])
    earthPoint.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(earthPoint.attRefOutMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg_I)
    mrpControl.rwParamsInMsg.subscribeTo(fswRwConfigMsg)
    mrpControl.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwConfigMsg)
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)

    for item in range(numRSA):
        saReference[item].attNavInMsg.subscribeTo(sNavObject.attOutMsg)
        saReference[item].attRefInMsg.subscribeTo(earthPoint.attRefOutMsg)
        saReference[item].hingedRigidBodyInMsg.subscribeTo(RSAList[item].spinningBodyOutMsg)
        saController[item].hingedRigidBodyInMsg.subscribeTo(RSAList[item].spinningBodyOutMsg)
        saController[item].hingedRigidBodyRefInMsg.subscribeTo(saReference[item].hingedRigidBodyRefOutMsg)


    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = simulationTime / simulationTimeStepFsw
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStepFsw, numDataPoints)
    snTransLog = sNavObject.transOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, snTransLog)
    snAttLog = sNavObject.attOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, snAttLog)
    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, attErrorLog)
    attRefLog = earthPoint.attRefOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, attRefLog)
    rwMotorLog = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, rwMotorLog)
    rwSpeedLog = rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, rwSpeedLog)
    mrpTorqueLog = mrpControl.cmdTorqueOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, mrpTorqueLog)
    zBACOLog = zBACObject.angOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, zBACOLog)
    yBACOLog = yBACObject.angOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, yBACOLog)
    xBACOLog = xBACObject.angOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, xBACOLog)
    dataRec = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, dataRec)
    dataEarth = ephemeris.ephemOutMsgs[1].recorder(samplingTime)
    scSim.AddModelToTask(dynTask, dataEarth)
    dataSun = ephemeris.ephemOutMsgs[0].recorder(samplingTime)
    scSim.AddModelToTask(dynTask, dataSun)


    zBACObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    zBACObject.celBodyInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[0])
    yBACObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    yBACObject.celBodyInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[1])
    xBACObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    xBACObject.celBodyInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[0])
    # A message is created that stores an array of the Omega wheel speeds
    rwLogs = []
    for item in range(numRW):
        rwLogs.append(rwStateEffector.rwOutMsgs[item].recorder(samplingTime))
        scSim.AddModelToTask(dynTask, rwLogs[item])

    saAngleLogs = []
    saRefAngleLogs = []
    for item in range(numRSA):
        saAngleLogs.append(RSAList[item].spinningBodyOutMsg.recorder(samplingTime))
        scSim.AddModelToTask(dynTask, saAngleLogs[item])
        saRefAngleLogs.append(saReference[item].hingedRigidBodyRefOutMsg.recorder(samplingTime))
        scSim.AddModelToTask(dynTask, saRefAngleLogs[item])





    # initialize Simulation:  This function runs the self_init()
    # cross_init() and reset() routines on each module.
    scSim.InitializeSimulation()

    # configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # retrieve the logged data

    timeData = snAttLog.times() * macros.NANO2HOUR
    dataSigmaBN = snAttLog.sigma_BN
    dataSigmaRN = attRefLog.sigma_RN  #plot sigma RN, polt sigma BN
    dataSigmaBR = attErrorLog.sigma_BR
    dataOmegaRW = rwSpeedLog.wheelSpeeds
    dataZMissAngle = zBACOLog.missAngle
    dataYMissAngle = yBACOLog.missAngle
    dataXMissAngle = xBACOLog.missAngle
    posData = dataRec.r_BN_N
    earthPos = dataEarth.r_BdyZero_N
    sunPos = dataSun.r_BdyZero_N
    SC_earth= earthPos- posData
    SC_sun= sunPos- posData

    SPE =SPE_angle(SC_sun,SC_earth)

    SC_earth_B=[]
    # check for triad
    r_BN_N = snTransLog.r_BN_N
    r_EN_N = earthPos
    eh_N = r_EN_N-r_BN_N
    sh_B = snAttLog.vehSunPntBdy
    sh_N = []
    for i in range(len(dataSigmaBN)):
        BN = rbk.MRP2C(dataSigmaBN[i])
        j= np.matmul(BN.T, sh_B[i])
        sh_N.append(j/np.linalg.norm(j))
        eh_N[i] =  eh_N[i]/np.linalg.norm(eh_N[i])
        k=np.matmul(BN,SC_earth[i])
        SC_earth_B.append(k/np.linalg.norm(k))



    a1_B = np.array(earthPoint.getA1Hat_B()).T
    h1_B = np.array(earthPoint.getH1Hat_B()).T



    RN = true_triad(eh_N[-1], sh_N[-1], a1_B, h1_B)
    check1= np.dot(np.array(RN[0,:]),sh_N[-1])
    check2= np.dot(RN[1,:],eh_N[-1])
    print("python triad:")
    print(check1.round(4))
    print(check2.round(4))
    print(RN)

    RN2= rbk.MRP2C(dataSigmaRN[-1])
    check3= np.dot(np.array(RN2[0,:]),sh_N[-1])
    check4= np.dot(RN2[1,:],eh_N[-1])
    print("triad:")
    print(check3.round(4))
    print(check4.round(4))
    print(RN2)

    dataRW = []
    for i in range(numRW):
        dataRW.append(rwLogs[i].u_current)


    dataAlpha = []
    dataAlphaRef = []
    for item in range(numRSA):
        dataAlpha.append(saAngleLogs[item].theta)
        dataAlphaRef.append(saRefAngleLogs[item].theta)


    # Plot the results
    figureList = {}
    plot_attitude(timeData, dataSigmaBN, dataSigmaRN, figID=1)
    pltName = fileName+"1"
    figureList[pltName] = plt.figure(1)
    plot_attitude_error(timeData, dataSigmaBR, figID=2)
    pltName = fileName+"2"
    figureList[pltName] = plt.figure(2)
    plot_rw_speeds(timeData, dataOmegaRW, numRW, figID=3)
    pltName = fileName+"3"
    figureList[pltName] = plt.figure(3)
    plot_solar_array_angle(timeData, dataAlpha, dataAlphaRef, figID=4)
    pltName = fileName+"4"
    figureList[pltName] = plt.figure(4)

    plot_solar_array_mis(timeData, dataAlpha, dataAlphaRef, figID=5)
    pltName = fileName+"5"
    figureList[pltName] = plt.figure(5)

    plot_Z_sun_angle(timeData, dataZMissAngle, figID=6)
    pltName = fileName+"6"
    figureList[pltName] = plt.figure(6)
    plot_Y_earth_angle(timeData, dataYMissAngle, figID=7)
    pltName = fileName+"7"
    figureList[pltName] = plt.figure(7)

    plot_x_sun_angle(timeData, dataXMissAngle, figID=8)
    pltName = fileName+"8"
    figureList[pltName] = plt.figure(8)


    plotOrbit(timeData, earthPos,'earth', figID=9)
    pltName = fileName+"9"
    figureList[pltName] = plt.figure(9)

    plotOrbit(timeData, sunPos,'sun', figID=10)
    pltName = fileName+"10"
    figureList[pltName] = plt.figure(10)

    plotOrbit(timeData, posData,'SC', figID=11)
    pltName = fileName+"11"
    figureList[pltName] = plt.figure(11)

    plotSPEAngle(timeData, SPE ,figID=12)
    pltName = fileName+"11"
    figureList[pltName] = plt.figure(12)

    if showPlots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


# Plotting functions
def plot_attitude(timeData, dataSigmaBN, dataSigmaRN, figID=None):
    """Plot the spacecraft attitude w.r.t. reference."""
    plt.figure(figID, figsize=(5, 2.75))
    for idx in range(3):
        plt.plot(timeData, dataSigmaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_{BN,' + str(idx + 1) + '}$')
    for idx in range(3):
        plt.plot(timeData, dataSigmaRN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3), linestyle='dashed',
                 label=r'$\sigma_{RN,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [hours]')
    plt.ylabel(r'Attitude $\sigma$')

def plot_attitude_error(timeData, dataSigmaBR, figID=None):
    """Plot the spacecraft attitude error."""
    plt.figure(figID, figsize=(5, 2.75))
    for idx in range(3):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx + 1) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [hours]')
    plt.ylabel(r'Attitude Tracking Error $\sigma_{B/R}$')

def plot_rw_speeds(timeData, dataOmegaRW, numRW, figID=None):
    """Plot the RW spin rates."""
    plt.figure(figID, figsize=(5, 2.75))
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\Omega_{' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [hours]')
    plt.ylabel('RW Speed (RPM) ')

def plot_solar_array_angle(timeData, dataAngle, dataRefAngle, figID=None):
    """Plot the solar array angles w.r.t references."""
    plt.figure(figID, figsize=(5, 2.75))
    for i, angle in enumerate(dataAngle):
        plt.plot(timeData, angle / np.pi * 180, color='C'+str(i), label=r'$\alpha_' + str(i+1) + '$')
    for i, angle in enumerate(dataRefAngle):
        plt.plot(timeData, angle / np.pi * 180, color='C'+str(i), linestyle='dashed', label=r'$\alpha_{R,' + str(i+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [hours]')
    plt.ylabel(r'Solar Array Angles [deg]')


def plot_solar_array_mis(timeData, dataAngle, dataRefAngle, figID=None):
    """Plot the solar array angles w.r.t references."""
    plt.figure(figID, figsize=(5, 2.75))
    for i, (angle1, angle2) in enumerate(zip(dataAngle, dataRefAngle)):
        plt.plot(timeData, abs(angle1-angle2) / np.pi * 180, color='C'+str(i), label=r'$\alpha_' + str(i+1) + '$')

    plt.legend(loc='lower right')
    plt.xlabel('Time [hours]')
    plt.ylabel(r'Solar Array Misalignment Angle [deg]')


def plot_Z_sun_angle(timeData, dataMissAngle, figID=None):
    """Plot the miss angle between sun sensor(s) boresight and Sun."""
    plt.figure(figID, figsize=(5, 2.75))

    data = dataMissAngle*macros.R2D
    for idx in range(1):
        plt.plot(timeData, data,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\alpha $')
    plt.legend(loc='lower right')
    plt.xlabel('Time [hours]')
    plt.ylabel('+Z/Sun angle (deg)')

def plot_x_sun_angle(timeData, dataMissAngle, figID=None):
    """Plot the miss angle between sun sensor(s) boresight and Sun."""
    plt.figure(figID, figsize=(5, 2.75))

    data = dataMissAngle*macros.R2D
    for idx in range(1):
        plt.plot(timeData, data,
                 color=unitTestSupport.getLineColor(idx+2, 3),
                 label=r'$\alpha $')
    plt.legend(loc='lower right')
    plt.xlabel('Time [hours]')
    plt.ylabel('X/Sun angle (deg)')

def plot_Y_earth_angle(timeData, dataMissAngle, figID=None):
    """Plot the miss angle between sun sensor(s) boresight and Sun."""
    plt.figure(figID, figsize=(5, 2.75))

    data = np.array(dataMissAngle)*macros.R2D
    for idx in range(1):
        plt.plot(timeData, data,
                 color=unitTestSupport.getLineColor(idx+1, 3),
                 label=r'$\alpha $')
    plt.legend(loc='lower right')
    plt.xlabel('Time [hours]')
    plt.ylabel('Y/earth angle (deg)')

def plotOrbit(timeData, posData,Object = 'SC' ,figID=None):
    # draw the inertial position vector components
    plt.figure(figID, figsize=(5, 2.75))
    fig = plt.gcf()

    for idx in range(3):
        plt.plot(timeData, posData[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [hours]')
    if Object =='SC':
        plt.ylabel('SC Inertial Position [km]')
    elif Object =='earth':
        plt.ylabel('Earth Inertial Position [km]')
    elif Object =='sun':
        plt.ylabel('Sun Inertial Position [km]')

    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

def plotSPEAngle(timeData, SPE ,figID=None):
    """Plot the miss angle between sun sensor(s) boresight and Sun."""
    plt.figure(figID, figsize=(5, 2.75))

    data = SPE
    for idx in range(1):
        plt.plot(timeData, data,
                 color=unitTestSupport.getLineColor(idx+1, 3),
                 label=r'$\alpha $')
    plt.legend(loc='lower right')
    plt.xlabel('Time [hours]')
    plt.ylabel('SPE angle (deg)')

def SPE_angle(list1, list2):
    list1, list2 = map(np.array, (list1, list2))
    angles=[]
    for v1, v2 in zip(list1, list2):
        dot_product = np.dot(v1, v2)
        cross_product = np.cross(v1, v2)

        # Compute angle in degrees
        angle = np.degrees(np.arccos(np.clip(dot_product / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1.0, 1.0)))
        # Adjust to [0, 360] range based on cross product sign
        if cross_product.any() < 0:
            angle = 360 - angle

        angles.append(angle)


    return angles




if __name__ == "__main__":
    run(
        True
    )
