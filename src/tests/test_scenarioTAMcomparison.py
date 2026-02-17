# SPDX-License-Identifier: ISC
# Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

"""
Xmera Scenario Script and Integrated Test

Purpose:  Test file for scenarioTAMcomparison
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
import scenarioTAMcomparison


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)

# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("orbitCase, useBias1, useBias2, useBounds1, useBounds2", [
    ('circular', False, False, True, True),
    ('elliptical', True, True, False, False),
    ('elliptical', False, False, False, False),
    ('circular', False, False, True, False)
])
@pytest.mark.scenarioTest

def test_scenarioTAMcomparison(show_plots, orbitCase, useBias1, useBias2, useBounds1, useBounds2):
    """This function is called by the py.test environment."""
    # each test method requires a single assert method to be called
    # provide a unique test method name, starting with test_

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    try:
        figureList = scenarioTAMcomparison.run(show_plots, orbitCase, useBias1, useBias2, useBounds1, useBounds2)
        # save the figures to the Doxygen scenario images folder
        for pltName, plt in list(figureList.items()):
            unitTestSupport.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount += 1
        testMessages.append("scenarioTAMcomparison  test are failed.")

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print(testFailCount)
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found

    assert testFailCount < 1, testMessages

if __name__ == "__main__":
    test_scenarioTAMcomparison(
        False,          # show_plots
        'circular',        # orbit case
        False,       # useBias1
        False,       # useBias2
        False,      # useBounds1
        False       # useBounds2
    )
