# SPDX-License-Identifier: ISC
# Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
"""Tests for how :mod:`AnalysisBaseClass` identifies the files in the results directory."""

import numpy as np
import pandas as pd
import pytest

from xmera.utilities.MonteCarlo.AnalysisBaseClass import (
    McAnalysisBaseClass,
    is_variable_data_file,
)


@pytest.mark.parametrize("name, expected", [
    ("attGuidMsg.sigma_BR.data", True),
    ("bskSat.totOrbEnergy.data", True),
    ("OrbitalElements.Omega_Capital.data", True),
    ("overrun.data", True),
    ("runtimeStats.data", True),
    ("MonteCarlo.data", False),
    ("run0.data", False),
    ("run137.data", False),
])
def test_classifies_results_directory_files(name, expected):
    """``is_variable_data_file`` accepts only per-variable frames as subset data."""
    assert is_variable_data_file(name) is expected


def test_classification_is_not_confused_by_the_results_directory_name(tmp_path):
    """The name of the results directory is mc_run_<timestamp>. This name contains 'run'.

    A match on the full path, not on the file name, made each file look like a per-run
    archive. This fault fully disabled the subset extraction and gave no warning.
    """
    results_dir = tmp_path / "mc_run_20240101-120000" / "results"
    assert is_variable_data_file(str(results_dir / "attGuidMsg.sigma_BR.data")) is True
    assert is_variable_data_file(str(results_dir / "run3.data")) is False


def make_variable_frame(n_runs, n_comp=3, n_times=4):
    times = np.arange(n_times, dtype=float) * 1e9
    columns = pd.MultiIndex.from_product(
        [range(n_runs), range(n_comp)], names=["runNum", "varIdx"])
    values = np.arange(n_times * n_runs * n_comp, dtype=float).reshape(n_times, -1)
    df = pd.DataFrame(values, index=times, columns=columns)
    df.index.name = "time[ns]"
    return df


@pytest.fixture()
def results_dir(tmp_path):
    """A results directory that contains two variables and the archives that the extraction must skip."""
    base = tmp_path / "mc_run_20240101-120000" / "results"
    base.mkdir(parents=True)
    for name in ("attGuidMsg.sigma_BR.data", "bskSat.totOrbEnergy.data"):
        pd.to_pickle(make_variable_frame(n_runs=4), str(base / name))
    pd.to_pickle({"messages": {}}, str(base / "MonteCarlo.data"))
    for run in range(4):
        pd.to_pickle({"messages": {}}, str(base / f"run{run}.data"))
    return base


def test_extract_subset_writes_only_requested_runs(results_dir):
    analysis = McAnalysisBaseClass()
    analysis.data_dir = str(results_dir)

    analysis.extract_subset_of_runs([(1,), (3,)])

    written = sorted(p.name for p in (results_dir / "subset").glob("*.data"))
    assert written == ["attGuidMsg.sigma_BR.data", "bskSat.totOrbEnergy.data"]

    subset = pd.read_pickle(str(results_dir / "subset" / "attGuidMsg.sigma_BR.data"))
    assert sorted(set(subset.columns.get_level_values(0))) == [1, 3]

    full = pd.read_pickle(str(results_dir / "attGuidMsg.sigma_BR.data"))
    pd.testing.assert_frame_equal(subset, full.loc[:, [1, 3]])


def test_extract_subset_skips_the_controller_and_per_run_archives(results_dir):
    """Archives are not frames. If the extraction reads an archive as a subset source, it raises an exception."""
    analysis = McAnalysisBaseClass()
    analysis.data_dir = str(results_dir)

    analysis.extract_subset_of_runs([(0,)])

    assert not (results_dir / "subset" / "MonteCarlo.data").exists()
    assert not (results_dir / "subset" / "run0.data").exists()


def test_extract_subset_is_skipped_when_already_populated(results_dir):
    """A second call for the same runs does not change the subset from the first call."""
    analysis = McAnalysisBaseClass()
    analysis.data_dir = str(results_dir)
    analysis.extract_subset_of_runs([(1,), (3,)])

    target = results_dir / "subset" / "attGuidMsg.sigma_BR.data"
    before = target.stat().st_mtime_ns

    analysis.extract_subset_of_runs([(1,), (3,)])

    assert target.stat().st_mtime_ns == before


def test_extract_subset_repopulates_when_a_different_subset_is_requested(results_dir):
    analysis = McAnalysisBaseClass()
    analysis.data_dir = str(results_dir)
    analysis.extract_subset_of_runs([(1,), (3,)])

    analysis.extract_subset_of_runs([(0,), (2,)])

    subset = pd.read_pickle(str(results_dir / "subset" / "attGuidMsg.sigma_BR.data"))
    assert sorted(set(subset.columns.get_level_values(0))) == [0, 2]
