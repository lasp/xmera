
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
import math
import os

import matplotlib.pyplot as plt
import numpy as np
import pytest
import spiceypy
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import oeStateEphem
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('fswAlgorithms')
from Basilisk import __path__
bskPath = __path__[0]

orbitPosAccuracy = 10000.0
orbitVelAccuracy = 1.0
colors = ['r','g','b']

@pytest.mark.parametrize('validChebyCurveTime, anomFlag', [
    (True, 0),
    (True, 1),
    (True, -1),
    (False, -1)
])
def test_chebyPosFitAllTest(show_plots, validChebyCurveTime, anomFlag):
    """Module Unit Test"""
    chebyPosFitAllTest(show_plots, validChebyCurveTime, anomFlag)

def test_zero_inputs(show_plots):
    """Module Unit Test"""
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    sim = SimulationBaseClass.SimBaseClass()

    FSWUnitTestProc = sim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    FSWUnitTestProc.addTask(sim.CreateNewTask(unitTaskName, macros.sec2nano(1)))

    oeStateModel = oeStateEphem.OEStateEphem()
    oeStateModel.modelTag = "oeStateModel"
    sim.AddModelToTask(unitTaskName, oeStateModel)

    oeStateModel.setCentralBodyGravitationalParameter(0)

    oeStateModel.setArcRadiusPeriapsisCoefficients(0, [0] * 20)
    oeStateModel.setArcEccentricityCoefficients(0, [0] * 20)
    oeStateModel.setArcInclinationCoefficients(0, [0] * 20)
    oeStateModel.setArcArgPeriapsisCoefficients(0, [0] * 20)
    oeStateModel.setArcTrueAnomalyCoefficients(0, [0] * 20)
    oeStateModel.setArcRaanCoefficients(0, [0] * 20)
    oeStateModel.setArcNumberOfCoefficients(0, 1)
    oeStateModel.setArcMiddleTime(0, 1)
    oeStateModel.setArcRadiusTime(0, 1/2.0)
    oeStateModel.setArcAnomalyFlag(0, 0)

    clockCorrData = messaging.TDBVehicleClockCorrelationMsgPayload()
    clockCorrData.vehicleClockTime = 0.0
    clockCorrData.ephemerisTime = oeStateModel.getArcMiddleTime(0) - oeStateModel.getArcRadiusTime(0)

    clockInMsg = messaging.TDBVehicleClockCorrelationMsg().write(clockCorrData)
    oeStateModel.clockCorrInMsg.subscribeTo(clockInMsg)

    dataLog = oeStateModel.stateFitOutMsg.recorder()
    sim.AddModelToTask(unitTaskName, dataLog)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(int(1*1.0E9))
    sim.ExecuteSimulation()

    posChebData = dataLog.r_BdyZero_N
    velChebData = dataLog.v_BdyZero_N

    np.testing.assert_allclose(posChebData, 0, atol=1e-10, err_msg="position values should have been zero")
    np.testing.assert_allclose(velChebData, 0, atol=1e-10, err_msg="velocity values should have been zero")

def chebyPosFitAllTest(show_plots, validChebyCurveTime, anomFlag):
    numCurvePoints = 4*8640+1
    curveDurationSeconds = 4*86400
    logPeriod = curveDurationSeconds // (numCurvePoints - 1)
    numberOfCoefficients = 14
    integFrame = "j2000"
    zeroBase = "Earth"
    centralBodyMu = 3.98574405096E14

    dateSpice = "2015 April 10, 00:00:00.0 TDB"
    spiceypy.furnsh(bskPath + '/supportData/EphemerisData/naif0012.tls')
    et = spiceypy.str2et(dateSpice)
    etStart = et
    etEnd = etStart + curveDurationSeconds

    spiceypy.furnsh(bskPath + '/supportData/EphemerisData/de430.bsp')
    spiceypy.furnsh(bskPath + '/supportData/EphemerisData/naif0012.tls')
    spiceypy.furnsh(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    spiceypy.furnsh(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    spiceypy.furnsh(path + '/TDRSS.bsp')

    tdrssPosList = []
    tdrssVelList = []
    timeHistory = np.linspace(etStart, etEnd, numCurvePoints)
    rpArray = []
    eccArray = []
    incArray = []
    OmegaArray = []
    omegaArray = []
    anomArray = []
    anomPrev = 0.0
    anomCount = 0

    for timeVal in timeHistory:
        stringCurrent = spiceypy.et2utc(timeVal, 'C', 4, 1024)
        [stateOut, _] = spiceypy.spkezr('-221', spiceypy.str2et(stringCurrent), integFrame, 'NONE', zeroBase)
        position = stateOut[0:3]*1000.0
        velocity = stateOut[3:6]*1000.0
        orbEl = orbitalMotion.rv2elem(centralBodyMu, position, velocity)
        tdrssPosList.append(position)
        tdrssVelList.append(velocity)
        rpArray.append(orbEl.rPeriap)
        eccArray.append(orbEl.e)
        incArray.append(orbEl.i)
        OmegaArray.append(orbEl.Omega)
        omegaArray.append(orbEl.omega)
        if anomFlag == 1:
            currentAnom = orbitalMotion.E2M(orbitalMotion.f2E(orbEl.f, orbEl.e), orbEl.e)
        else:
            currentAnom = orbEl.f
        if currentAnom < anomPrev:
            anomCount += 1
        anomArray.append(2*math.pi*anomCount + currentAnom)
        anomPrev = currentAnom

    tdrssPosList = np.array(tdrssPosList)
    tdrssVelList = np.array(tdrssVelList)

    fitTimes = np.linspace(-1, 1, numCurvePoints)
    chebRpCoeff = np.polynomial.chebyshev.chebfit(fitTimes, rpArray, numberOfCoefficients - 1) # np chebfit takes in the degree, not the number of coefficients
    chebEccCoeff = np.polynomial.chebyshev.chebfit(fitTimes, eccArray, numberOfCoefficients - 1)
    chebIncCoeff = np.polynomial.chebyshev.chebfit(fitTimes, incArray, numberOfCoefficients - 1)
    chebOmegaCoeff = np.polynomial.chebyshev.chebfit(fitTimes, OmegaArray, numberOfCoefficients - 1)
    chebomegaCoeff = np.polynomial.chebyshev.chebfit(fitTimes, omegaArray, numberOfCoefficients - 1)
    chebAnomCoeff = np.polynomial.chebyshev.chebfit(fitTimes, anomArray, numberOfCoefficients - 1)

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    sim = SimulationBaseClass.SimBaseClass()

    FSWUnitTestProc = sim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    FSWUnitTestProc.addTask(sim.CreateNewTask(unitTaskName, macros.sec2nano(logPeriod)))

    oeStateModel = oeStateEphem.OEStateEphem()
    oeStateModel.modelTag = "oeStateModel"
    sim.AddModelToTask(unitTaskName, oeStateModel)

    oeStateModel.setCentralBodyGravitationalParameter(centralBodyMu)

    oeStateModel.setArcRadiusPeriapsisCoefficients(0, chebRpCoeff.tolist() + [0] * (20 - numberOfCoefficients))
    oeStateModel.setArcEccentricityCoefficients(0, chebEccCoeff.tolist() + [0] * (20 - numberOfCoefficients))
    oeStateModel.setArcInclinationCoefficients(0, chebIncCoeff.tolist() + [0] * (20 - numberOfCoefficients))
    oeStateModel.setArcArgPeriapsisCoefficients(0, chebomegaCoeff.tolist() + [0] * (20 - numberOfCoefficients))
    oeStateModel.setArcTrueAnomalyCoefficients(0, chebAnomCoeff.tolist() + [0] * (20 - numberOfCoefficients))
    oeStateModel.setArcRaanCoefficients(0, chebOmegaCoeff.tolist() + [0] * (20 - numberOfCoefficients))
    oeStateModel.setArcNumberOfCoefficients(0, numberOfCoefficients)
    oeStateModel.setArcMiddleTime(0, etStart + curveDurationSeconds/2.0)
    oeStateModel.setArcRadiusTime(0, curveDurationSeconds/2.0)

    if not (anomFlag == -1):
        oeStateModel.setArcAnomalyFlag(0, anomFlag)

    clockCorrData = messaging.TDBVehicleClockCorrelationMsgPayload()
    clockCorrData.vehicleClockTime = 0.0
    clockCorrData.ephemerisTime = oeStateModel.getArcMiddleTime(0) - oeStateModel.getArcRadiusTime(0)

    clockInMsg = messaging.TDBVehicleClockCorrelationMsg().write(clockCorrData)
    oeStateModel.clockCorrInMsg.subscribeTo(clockInMsg)

    dataLog = oeStateModel.stateFitOutMsg.recorder()
    sim.AddModelToTask(unitTaskName, dataLog)

    if not validChebyCurveTime:
        sim.InitializeSimulation()
        # increase the run time by one logging period so that the sim time is outside the
        # valid chebychev curve duration
        sim.ConfigureStopTime(int((curveDurationSeconds + logPeriod) * 1.0E9))
        sim.ExecuteSimulation()
    else:
        sim.InitializeSimulation()
        sim.ConfigureStopTime(int(curveDurationSeconds*1.0E9))
        sim.ExecuteSimulation()

    posChebData = dataLog.r_BdyZero_N
    velChebData = dataLog.v_BdyZero_N

    if not validChebyCurveTime:
        lastLogidx = (curveDurationSeconds + logPeriod) // logPeriod - 1
        secondLastPos = posChebData[lastLogidx + 1, 0:] - tdrssPosList[lastLogidx, :]
        lastPos = posChebData[lastLogidx, 0:] - tdrssPosList[lastLogidx, :]

        np.testing.assert_array_equal(secondLastPos, lastPos, "Expected Chebychev position to rail high or low")

        secondLastVel = velChebData[lastLogidx + 1, 0:] - tdrssVelList[lastLogidx, :]
        lastVel = velChebData[lastLogidx, 0:] - tdrssVelList[lastLogidx, :]
        np.testing.assert_array_equal(secondLastVel, lastVel, "Expected Chebychev velocity to rail high or low")

    else:
        maxErrVec = [abs(max(posChebData[:, 0] - tdrssPosList[:, 0])),
                     abs(max(posChebData[:, 1] - tdrssPosList[:, 1])),
                     abs(max(posChebData[:,2] - tdrssPosList[:, 2]))]
        maxVelErrVec = [abs(max(velChebData[:, 0] - tdrssVelList[:, 0])),
                        abs(max(velChebData[:, 1] - tdrssVelList[:, 1])),
                        abs(max(velChebData[:, 2] - tdrssVelList[:, 2]))]

        np.testing.assert_array_less(max(maxErrVec), orbitPosAccuracy, "maxErrVec >= orbitPosAccuracy")
        np.testing.assert_array_less(max(maxVelErrVec), orbitVelAccuracy, "maxVelErrVec >= orbitVelAccuracy")

        plt.close("all")
        # plot the fitted and actual position coordinates
        plt.figure(1)
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='plain')
        for idx in range(0, 3):
            plt.plot(dataLog.times()*macros.NANO2HOUR,
                     posChebData[:, idx]/1000,
                     color=colors[idx],
                     linewidth=0.5,
                     label='$r_{fit,' + str(idx) + '}$')
            plt.plot(dataLog.times()*macros.NANO2HOUR,
                     tdrssPosList[:, idx]/1000,
                     color=colors[idx],
                     linestyle='dashed', linewidth=2,
                     label='$r_{true,' + str(idx) + '}$')
        plt.legend(loc='lower right')
        plt.xlabel('Time [h]')
        plt.ylabel('Inertial Position [km]')

        # plot the fitted and actual velocity coordinates
        plt.figure(2)
        for idx in range(0, 3):
            plt.plot(dataLog.times()*macros.NANO2HOUR,
                     velChebData[:, idx]/1000,
                     color=colors[idx],
                     linewidth=0.5,
                     label='$v_{fit,' + str(idx) + '}$')
            plt.plot(dataLog.times()*macros.NANO2HOUR,
                     tdrssVelList[:, idx]/1000,
                     color=colors[idx],
                     linestyle='dashed', linewidth=2,
                     label='$v_{true,' + str(idx) + '}$')
        plt.legend(loc='lower right')
        plt.xlabel('Time [h]')
        plt.ylabel('Velocity [km/s]')

        # plot the difference in position coordinates
        plt.figure(3)
        arrayLength = posChebData[:, 0].size
        for idx in range(0,3):
            plt.plot(dataLog.times() * macros.NANO2HOUR,
                     posChebData[:, idx] - tdrssPosList[:, idx],
                     color=colors[idx],
                     linewidth=0.5,
                     label=r'$\Delta r_{' + str(idx) + '}$')
        plt.plot(dataLog.times() * macros.NANO2HOUR,
                 orbitPosAccuracy*np.ones(arrayLength),
                 color='r',
                 linewidth=1)
        plt.plot(dataLog.times() * macros.NANO2HOUR,
                 -orbitPosAccuracy * np.ones(arrayLength),
                 color='r',
                 linewidth=1)
        plt.legend(loc='lower right')
        plt.xlabel('Time [h]')
        plt.ylabel('Position Difference [m]')

        # plot the difference in velocity coordinates
        plt.figure(4)
        arrayLength = velChebData[:, 0].size
        for idx in range(0,3):
            plt.plot(dataLog.times() * macros.NANO2HOUR,
                     velChebData[:, idx] - tdrssVelList[:, idx],
                     color=colors[idx],
                     linewidth=0.5,
                     label=r'$\Delta v_{' + str(idx) + '}$')
        plt.plot(dataLog.times() * macros.NANO2HOUR,
                 orbitVelAccuracy*np.ones(arrayLength),
                 color='r',
                 linewidth=1)
        plt.plot(dataLog.times() * macros.NANO2HOUR,
                 -orbitVelAccuracy * np.ones(arrayLength),
                 color='r',
                 linewidth=1)
        plt.legend(loc='lower right')
        plt.xlabel('Time [h]')
        plt.ylabel('Velocity Difference [m/s]')

    if show_plots:
        plt.show()
        plt.close('all')



if __name__ == "__main__":
    chebyPosFitAllTest(True,        # showPlots
                       True,        # validChebyCurveTime
                       1)           # anomFlag
