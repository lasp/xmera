# SPDX-License-Identifier: ISC
# Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

"""
Xmera Scenario Script and Integrated Test

Purpose:  Integrated test of the spacecraft(), RWs, simpleNav() and mrpFeedback()
and boreAngCalc() modules.  Illustrates the violation of several keep-in and
keep out constraints while performing a slew maneuver with RWs as actuators.
"""

import inspect
import os
import sys

import pytest
from xmera.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioAttitudeConstraintViolation


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parameterize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.


@pytest.mark.parametrize("use2SunSensors, starTrackerFov, sunSensorFov, attitudeSetCase", [(False, 20, 70, 0),
                                                                                           (True,  20, 70, 0),
                                                                                           (True,  20, 70, 1),
																						   (True,  20, 70, 2),
																						   (True,  30, 70, 3)])
@pytest.mark.scenarioTest

# provide a unique test method name, starting with test_
def test_bskAttitudeConstraintViolation(show_plots, use2SunSensors, starTrackerFov, sunSensorFov, attitudeSetCase):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    try:
        figureList = scenarioAttitudeConstraintViolation.run(show_plots, use2SunSensors, starTrackerFov, sunSensorFov, attitudeSetCase)
        # save the figures to the Doxygen scenario images folder
        for pltName, plt in list(figureList.items()):
            unitTestSupport.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount += 1
        testMessages.append("scenarioAttitudeConstraintViolation tests have failed.")

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print(testFailCount)
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages
