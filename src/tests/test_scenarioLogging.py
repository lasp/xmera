# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import inspect
import os
import sys

import pytest

from xmera.architecture import sim_model

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioLogging


@pytest.mark.parametrize("case, firstShown, secondShown", [
    (0, True, True),
    (1, False, False),
    (2, False, True),
])
@pytest.mark.scenarioTest
def test_scenarioLogging(tmp_path, case, firstShown, secondShown):
    """The log level of each case decides which module messages are in the log file."""
    logFile = tmp_path / "scenario.log"
    # Other tests in the same process can leave a different log level set.
    sim_model.reset_log_levels()
    try:
        scenarioLogging.run(case, str(logFile))
    finally:
        # Replacing the sink closes the log file. Restore the defaults for the other tests.
        sim_model.set_log_sink("stdout")
        sim_model.reset_log_levels()

    text = logFile.read_text()
    assert ("[cppModuleTemplate1]" in text) == firstShown, text
    assert ("[cppModuleTemplate2]" in text) == secondShown, text
