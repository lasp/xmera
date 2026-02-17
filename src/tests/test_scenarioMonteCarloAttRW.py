# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

"""
Xmera Integrated Test

Purpose:  Integrated test of the MonteCarlo module.  Runs multiple
scenarioAttitudeFeedbackRW with dispersed initial parameters
"""

import inspect
import os
import sys

import pytest

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples/')
import scenarioMonteCarloAttRW


# Run initial conditions and plot with matplotlib
@pytest.mark.parametrize("MCCases",
                         [1,2])
@pytest.mark.slowtest
@pytest.mark.scenarioTest
def test_MonteCarloSimulation(show_plots, MCCases):
    """This function is called by the py.test environment."""
    # each test method requires a single assert method to be called
    scenarioMonteCarloAttRW.run(True, MCCases , show_plots)
    return
