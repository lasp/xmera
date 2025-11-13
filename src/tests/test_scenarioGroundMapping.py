import inspect
import os
import sys

import pytest
from xmera.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioGroundMapping
@pytest.mark.scenarioTest

def test_scenarioGroundMapping(show_plots):
    """This function is called by the py.test environment."""

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    # each test method requires a single assert method to be called
    try:
        figureList = scenarioGroundMapping.run(False, True)

        # save the figures to the Sphinx scenario images folder
        for pltName, plt in list(figureList.items()):
            unitTestSupport.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount += 1
        testMessages.append("Ground mapping failed.")

    assert testFailCount < 1, testMessages

    return
