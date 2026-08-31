# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

"""
Xmera Scenario Script and Integrated Test

Purpose:  Integrated test of the spacecraft() and gravity modules illustrating
a four body system, for a Patched Conics analysis of an interplanetary transfer
between Earth and Jupiter
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

import scenarioPatchedConics

@pytest.mark.skip(reason="???")
@pytest.mark.scenarioTest
@pytest.mark.slowtest



# provide a unique test method name, starting with test_
def test_scenarioPatchedConics(show_plots):
    """This function is called by the py.test environment."""

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    try:
        dataPos, figureList = scenarioPatchedConics.run(show_plots)
        # save the figures to the Doxygen scenario images folder
        for pltName, plt in list(figureList.items()):
            unitTestSupport.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount += 1
        testMessages.append("scenarioPatchedConics test are failed.")

    # setup truth data for unit test
    truePos = [
        [19503460698.246426, 1948347074.142926, 0.0]
    ]


    # compare the results to the truth values
    accuracy = 1000000.0 # meters
    testFailCount, testMessages = unitTestSupport.compareArray(
        truePos, dataPos, accuracy, "r_BN_N Vector",testFailCount, testMessages)

    print(truePos, dataPos)

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print("testFailCount: " + str(testFailCount))
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages
