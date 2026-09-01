# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

"""
Coarse Sun Sensor Unit Test

Purpose:  Test the proper function of the coarse sun sensor (css) module.
For basic functionality, results are compared to simple truth values calculated using np.cos().
For noise testing, noiseless truth values are subtracted from the output and the standard deviation is compared
to the input standard deviation.
For css constellation set up, two identical constellations are set up with different methods and compared to
each other
"""

import os

import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.simulation import coarseSunSensor
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros
from xmera.utilities import orbitalMotion as om
from xmera.utilities import unitTestSupport

path = os.path.dirname(os.path.abspath(__file__))

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.skip(reason="Test success is architecture-dependent due to floating-point deviations")
@pytest.mark.parametrize(
    "cssFault",
    [
        "CSSFAULT_OFF",
        "CSSFAULT_STUCK_CURRENT",
        "CSSFAULT_STUCK_MAX",
        "CSSFAULT_STUCK_RAND",
        "CSSFAULT_RAND",
    ])
# provide a unique test method name, starting with test_
def test_coarseSunSensor(cssFault):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(cssFault)
    assert testResults < 1, testMessage

def run(cssFault):
    # np.random.seed(10)

    testFailCount = 0
    testMessages = []
    testTaskName = "unitTestTask"
    testProcessName = "unitTestProcess"
    testTaskRate = macros.sec2nano(0.1)

    # Create a simulation container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # unitTestSim.RNGSeed = 10

    # Ensure simulation is empty
    testProc = unitTestSim.CreateNewProcess(testProcessName)
    testProc.addTask(testTaskName, testTaskRate)

    # Input Message Setup
    # Creates inputs from sun, spacecraft, and eclipse so that those modules don't have to be included
    # Create dummy sun message
    sunPositionMsg = messaging.SpicePlanetStateMsgPayload()
    sunPositionMsg.PositionVector = [om.AU * 1000.0, 0.0, 0.0]
    sunMsg = messaging.SpicePlanetStateMsg().write(sunPositionMsg)

    # Create dummy spacecraft message
    satelliteStateMsg = messaging.SCStatesMsgPayload()
    satelliteStateMsg.r_BN_N = [0.0, 0.0, 0.0]
    angle = np.pi/16
    satelliteStateMsg.sigma_BN = [0., 0., angle]
    scMsg = messaging.SCStatesMsg().write(satelliteStateMsg)

    # Calculate sun distance factor
    CSS = coarseSunSensor.CoarseSunSensor()

    CSS.fov = 80. * macros.D2R         # half-angle field of view value
    CSS.scaleFactor = 2.0
    CSS.nHat_B = np.array([1., 0., 0.])
    CSS.sunInMsg.subscribeTo(sunMsg)
    CSS.stateInMsg.subscribeTo(scMsg)
    CSS.modelTag = "CSS"
    CSS.RNGSeed = 123
    unitTestSim.AddModelToTask(testTaskName, CSS)

    # log single CSS
    cssRecoder = CSS.cssDataOutMsg.recorder()
    unitTestSim.AddModelToTask(testTaskName, cssRecoder)

    # Truth Values
    if cssFault == "CSSFAULT_OFF":
        cssFaultValue = coarseSunSensor.CSSFAULT_OFF
        truthValue = 0.0
    elif cssFault == "CSSFAULT_STUCK_CURRENT":
        cssFaultValue = coarseSunSensor.CSSFAULT_STUCK_CURRENT
        truthValue = 1.4280970791070948
    elif cssFault == "CSSFAULT_STUCK_MAX":
        cssFaultValue = coarseSunSensor.CSSFAULT_STUCK_MAX
        truthValue = 2.0
    elif cssFault == "CSSFAULT_STUCK_RAND":
        cssFaultValue = coarseSunSensor.CSSFAULT_STUCK_RAND
        truthValue = 1.7278304838858731
    elif cssFault == "CSSFAULT_RAND":
        cssFaultValue = coarseSunSensor.CSSFAULT_RAND
        truthValue = 0.7974448327854251
    else:
        NotImplementedError("Fault type specified does not exist.")

    unitTestSim.InitializeSimulation()

    # Execute the simulation for one time step
    unitTestSim.singleStepProcesses()
    CSS.faultState = cssFaultValue
    for i in range(3):
        unitTestSim.singleStepProcesses()

    cssOutput = cssRecoder.OutputData[-1]
    print(cssOutput)
    print(truthValue)

    if cssFault == "CSSFAULT_OFF":
        if not truthValue == cssOutput:
            testFailCount += 1
    elif not unitTestSupport.isDoubleEqualRelative(cssOutput, truthValue, 1E-12):
        testFailCount += 1

    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    run("CSSFAULT_STUCK_MAX")
