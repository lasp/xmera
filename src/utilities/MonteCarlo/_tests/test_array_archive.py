# SPDX-License-Identifier: ISC
# Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
"""Tests for the preallocated array archive of :mod:`ArrayArchive` and its data writer.

The writer runs on the calling thread with a standard :class:`queue.Queue`, as in test_data_writer.
"""

import queue

import numpy as np
import pytest

from xmera.utilities.MonteCarlo import ArrayArchive
from xmera.utilities.MonteCarlo.Controller import Controller

SENTINEL = (None, None, True)
DT = 1_000_000_000


def item(times, values):
    return np.column_stack([np.asarray(times, dtype=float), np.asarray(values, dtype=float)])


def run_data(value, n_steps=4, components=3, name="msg.var"):
    times = np.arange(n_steps) * DT
    values = np.full((n_steps, components), float(value))
    return {"messages": {name: item(times, values)}, "variables": {}, "custom": {}}


def run_writer(log_dir, runs, var_cast=None, rows=1):
    q = queue.Queue()
    for run_index, data in runs:
        q.put((data, run_index, None))
    q.put(SENTINEL)
    writer = ArrayArchive.ArrayDataWriter(q, rows=rows)
    writer.set_log_dir(str(log_dir))
    writer.set_var_cast(var_cast)
    writer.run()


def test_each_run_is_written_into_its_own_row(tmp_path):
    run_writer(tmp_path, [(i, run_data(i)) for i in (2, 0, 1)], rows=3)
    time_ns, runs, values = ArrayArchive.read(tmp_path, "msg.var")
    assert list(runs) == [0, 1, 2]
    assert values.shape == (3, 3, 4)
    np.testing.assert_array_equal(time_ns, np.arange(4) * DT)
    np.testing.assert_array_equal(values[:, 0, 0], [0.0, 1.0, 2.0])
    assert isinstance(values, np.memmap)


def test_var_cast_float_stores_single_precision(tmp_path):
    run_writer(tmp_path, [(0, run_data(0.1))], var_cast="float")
    assert ArrayArchive.read(tmp_path, "msg.var")[2].dtype == np.float32


def test_rerun_indexes_take_free_rows_and_read_in_run_order(tmp_path):
    run_writer(tmp_path, [(16, run_data(16)), (7, run_data(7))], rows=2)
    _, runs, values = ArrayArchive.read(tmp_path, "msg.var")
    assert list(runs) == [7, 16]
    np.testing.assert_array_equal(values[:, 0, 0], [7.0, 16.0])


def test_more_runs_than_rows_grow_the_item(tmp_path):
    run_writer(tmp_path, [(i, run_data(i)) for i in range(5)], rows=2)
    _, runs, values = ArrayArchive.read(tmp_path, "msg.var")
    assert list(runs) == list(range(5))
    np.testing.assert_array_equal(values[:, 2, -1], np.arange(5.0))


def test_a_longer_later_run_widens_the_time_grid(tmp_path):
    run_writer(tmp_path, [(0, run_data(1.0, n_steps=3)), (1, run_data(2.0, n_steps=5))], rows=2)
    time_ns, _, values = ArrayArchive.read(tmp_path, "msg.var")
    assert len(time_ns) == 5
    assert np.isnan(values[0, 0, 3:]).all()
    np.testing.assert_array_equal(values[0, 0, :3], 1.0)
    np.testing.assert_array_equal(values[1, 0], 2.0)


def test_fewer_components_are_nan(tmp_path):
    run_writer(tmp_path, [(0, run_data(1.0, components=3)), (1, run_data(2.0, components=2))], rows=2)
    values = ArrayArchive.read(tmp_path, "msg.var")[2]
    assert np.isnan(values[1, 2]).all()


def test_an_item_that_is_not_a_numeric_time_series_is_not_archived(tmp_path):
    times = np.arange(3) * DT
    structs = np.empty((3, 2), dtype=object)
    structs[:, 0] = times
    structs[:, 1] = [object() for _ in range(3)]
    data = {"messages": {"msg.packets": structs, "msg.var": item(times, np.zeros(3))}, "variables": {}}
    run_writer(tmp_path, [(0, data)])
    assert ArrayArchive.item_names(tmp_path) == ["msg.var"]


def test_an_interrupted_run_leaves_its_row_free(tmp_path):
    run_writer(tmp_path, [(0, run_data(0.0)), (1, run_data(1.0))], rows=2)
    directory = ArrayArchive.item_directory(tmp_path, "msg.var")
    runs = np.load(f"{directory}/{ArrayArchive.RUNS}")
    runs[1] = -1
    np.save(f"{directory}/{ArrayArchive.RUNS}", runs)
    assert ArrayArchive.runs_in(tmp_path, "msg.var") == {0}
    run_writer(tmp_path, [(1, run_data(5.0))])
    np.testing.assert_array_equal(ArrayArchive.read(tmp_path, "msg.var")[2][:, 0, 0], [0.0, 5.0])


def test_retained_data_gives_back_a_run_as_the_retention_policy_did(tmp_path):
    data = run_data(3.0)
    data["variables"] = {"sat.energy": item(np.arange(4) * DT, np.arange(4.0))}
    run_writer(tmp_path, [(0, run_data(1.0)), (1, data)], rows=2)
    back = ArrayArchive.retained_data(tmp_path, 1)
    assert back["index"] == 1
    np.testing.assert_array_equal(back["messages"]["msg.var"], data["messages"]["msg.var"])
    np.testing.assert_array_equal(back["variables"]["sat.energy"], data["variables"]["sat.energy"])


def test_controller_rejects_an_unknown_archive_format():
    with pytest.raises(ValueError):
        Controller().set_archive_format("csv")
