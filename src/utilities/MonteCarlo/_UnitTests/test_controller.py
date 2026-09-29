# SPDX-License-Identifier: ISC
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
"""Unit tests for Controller orchestration that do not execute a simulation."""

import json
import os

import pytest

from xmera.utilities.MonteCarlo import Controller as controller_module
from xmera.utilities.MonteCarlo.Controller import Controller


class _RecordingJobRunner:
    """Replaces JobRunner and records its constructor arguments."""

    instances = []

    def __init__(self, results_dir, var_cast=None):
        self.results_dir = results_dir
        self.var_cast = var_cast
        self.queue = None
        _RecordingJobRunner.instances.append(self)

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        return False


@pytest.fixture
def no_jobs(monkeypatch):
    """Replace the job runner and the job driver, so that no worker starts."""
    _RecordingJobRunner.instances = []
    monkeypatch.setattr(controller_module, "JobRunner", _RecordingJobRunner)
    monkeypatch.setattr(Controller, "_drive_jobs", lambda self, *args, **kwargs: [])
    return _RecordingJobRunner.instances


def _write_initial_conditions(directory, run_indexes):
    os.makedirs(directory, exist_ok=True)
    for index in run_indexes:
        with open(os.path.join(directory, f"run{index}.json"), "w") as f:
            json.dump({}, f)


def test_run_initial_conditions_passes_var_cast(tmp_path, no_jobs):
    ic_directory = tmp_path / "old" / "initial_conditions"
    _write_initial_conditions(ic_directory, [0, 2])

    mc = Controller()
    mc.archive_dir = str(tmp_path / "new")
    mc.set_var_cast("float")
    mc.run_initial_conditions([0, 2], str(ic_directory))

    assert [runner.var_cast for runner in no_jobs] == ["float"]
