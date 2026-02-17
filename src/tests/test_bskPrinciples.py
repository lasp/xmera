# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

"""
Integrated tests

Purpose:  This script calls a series of quick start guide demonstration scripts to ensure
that they complete properly.
"""

import fnmatch
import importlib
import inspect
import os
import sys

import pytest
import xmera.architecture.messaging
from xmera.architecture import sim_model
from xmera.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../docs/source/codeSamples')
files = fnmatch.filter(os.listdir(path + '/../../docs/source/codeSamples'), "*.py")

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("bskScript", files)
@pytest.mark.scenarioTest
def test_scenarioBskPrinciples(show_plots, bskScript):

    sim_model.setDefaultLogLevel(sim_model.BSK_WARNING)
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    # import the bskSim script to be tested
    scene_plt = importlib.import_module(os.path.splitext(bskScript)[0])

    try:
        figureList = scene_plt.run()

        # save the figures to the RST scenario images folder
        if(figureList and figureList != {}):
            for pltName, plt in list(figureList.items()):
                unitTestSupport.saveScenarioFigure(pltName, plt, path)
    except OSError as err:
        testFailCount = testFailCount + 1
        testMessages.append("OS error: {0}".format(err))

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found

    assert testFailCount < 1, testMessages


if __name__ == "__main__":
    test_scenarioBskPrinciples(
        False,        # show_plots
        'bsk-4'
    )
