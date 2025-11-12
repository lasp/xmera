#
# Xmera Integrated Test
#
# Purpose:  Integrated test of the MonteCarlo module with Spice usage.
#

import inspect
import os
import platform
import sys

import pytest

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples/')
import scenarioMonteCarloSpice

@pytest.mark.skipif(sys.version_info < (3, 9)  and platform.system() == 'Darwin',
                    reason="Test has issues with Controller class and older python.")
@pytest.mark.scenarioTest
def test_MonteCarloSimulationDatashader(show_plots):
    """This function is called by the py.test environment."""

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    # each test method requires a single assert method to be called
    try:
        scenarioMonteCarloSpice.run()
    except OSError as err:
        testFailCount += 1
        testMessages.append("MC Spice tutorial failed.")

    assert testFailCount < 1, testMessages

    return
