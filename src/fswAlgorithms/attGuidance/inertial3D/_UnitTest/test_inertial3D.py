#
#  ISC License
#
#  Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.fswAlgorithms import inertial3D                   # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.architecture import messaging


@pytest.mark.parametrize("set_SigmaRN", [True, False])
def test_inertial3D(show_plots, set_SigmaRN):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    module = inertial3D.Inertial3D()
    module.modelTag = "inertial3D"

    if set_SigmaRN:
        vector = np.random.rand(3)
        module.setSigmaR0N(vector)
    else:
        vector = [0.0, 0.0, 0.0]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.attRefOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # retrieve the module output
    sigma_RN = dataLog.sigma_RN
    omega_RN_N = dataLog.omega_RN_N
    domega_RN_N = dataLog.domega_RN_N

    # set the filtered output truth states
    sigma_RN_truth = [vector] * 3
    omega_RN_N_truth = [[0.0, 0.0, 0.0]] * 3
    domega_RN_N_truth = [[0.0, 0.0, 0.0]] * 3

    # compare the module results to the truth values
    accuracy = 1e-12

    np.testing.assert_allclose(sigma_RN, sigma_RN_truth, rtol=0, atol=accuracy, verbose=True)
    np.testing.assert_allclose(omega_RN_N, omega_RN_N_truth, rtol=0, atol=accuracy, verbose=True)
    np.testing.assert_allclose(domega_RN_N, domega_RN_N_truth, rtol=0, atol=accuracy, verbose=True)


if __name__ == "__main__":
    test_inertial3D(False)
