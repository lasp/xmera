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


class _Job:
    def __init__(self, index):
        self.index = index


class _FailingPool:
    """Replaces mp.Pool. It reports the first job as a success, then raises an unexpected error."""

    def __init__(self, num_processes):
        pass

    def imap_unordered(self, function, jobs):
        yield True, jobs[0][0].index, "", ""
        raise RuntimeError("pool stopped")

    def close(self):
        pass

    def terminate(self):
        pass

    def join(self):
        pass


def test_record_unfinished_uses_the_job_indexes():
    failures = []
    Controller._record_unfinished(failures, {4}, [4, 9, 12], "KeyboardInterrupt")
    assert [f.run_index for f in failures] == [9, 12]
    assert {f.exception_type for f in failures} == {"KeyboardInterrupt"}


def test_pool_error_records_the_unfinished_non_consecutive_runs(monkeypatch):
    monkeypatch.setattr(controller_module.mp, "Pool", _FailingPool)
    jobs = (_Job(index) for index in [4, 9, 12])

    failures = Controller()._drive_jobs(jobs, 3, None, num_processes=2)

    assert [f.run_index for f in failures] == [9, 12]
    assert {f.exception_type for f in failures} == {"RuntimeError"}


def test_execute_simulations_without_archive_dir_raises(no_jobs):
    mc = Controller()
    mc.set_execution_count(1)
    with pytest.raises(ValueError, match="archive_dir"):
        mc.execute_simulations()


def test_run_initial_conditions_without_archive_dir_raises(tmp_path, no_jobs):
    ic_directory = tmp_path / "initial_conditions"
    _write_initial_conditions(ic_directory, [0])
    with pytest.raises(ValueError, match="archive_dir"):
        Controller().run_initial_conditions([0], str(ic_directory))


def test_re_run_cases_without_a_run_raises(no_jobs):
    with pytest.raises(ValueError, match="Controller.load"):
        Controller().re_run_cases([0])
