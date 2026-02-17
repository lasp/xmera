# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import inspect
import os
import sys

import pytest
from xmera.architecture import sim_model
from xmera.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioCustomGravBody as testScenario

@pytest.mark.scenarioTest

def test_simplePowerDemo(show_plots):
    """This function is called by the py.test environment."""

    # suppress printing out BSK_INFORMATION states
    sim_model.setDefaultLogLevel(sim_model.BSK_WARNING)

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    # each test method requires a single assert method to be called
    try:
        figureList = testScenario.run(False)

        # save the figures to the Doxygen scenario images folder
        for pltName, plt in list(figureList.items()):
            unitTestSupport.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount += 1
        testMessages.append("Data file to Viz tutorial failed.")

    assert testFailCount < 1, testMessages

    return
