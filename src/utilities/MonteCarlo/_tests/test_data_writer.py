# SPDX-License-Identifier: ISC
# Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
"""Characterization tests for :class:`DataWriter`'s on-disk output.

These tests record the behavior of the pickle-based writer at this time. When the writer
changes to a preallocated array store, these tests can show that the behavior stays the
same. Each test asserts what the code *does*, not what the code must do. If the behavior
at this time causes a loss of data, the test name and the docstring identify the loss.

:class:`DataWriter` is an ``mp.Process`` subclass, but ``run()`` uses only
``self._queue.get()``. Thus these tests supply a standard :class:`queue.Queue` to the writer
on the calling thread. The tests stay in the fast suite because they use no worker
processes, no simulation, and no drain timeout.
"""

import logging
import pickle
import queue

import numpy as np
import pandas as pd
import pytest

from xmera.utilities.MonteCarlo.DataWriter import DataWriter


SENTINEL = (None, None, True)


def make_item(times, values):
    """Make a retained-data array with the same shape as the output of the retention policy.

    Column 0 is the sample time and the other columns are the payload. This layout is the
    same as the layout of ``unitTestSupport.addTimeColumn`` and ``PythonVariableLogger.GetData``.
    """
    return np.column_stack([np.asarray(times, dtype=float), np.asarray(values, dtype=float)])


def run_writer(log_dir, runs, var_cast=None):
    """Send ``runs`` through a DataWriter on the calling thread and return the log directory.

    :param runs: iterable of ``(run_index, retained_data_dict)`` in arrival order.
    """
    q = queue.Queue()
    for run_index, data in runs:
        q.put((data, run_index, None))
    q.put(SENTINEL)

    writer = DataWriter(q)
    writer.set_log_dir(str(log_dir))
    writer.set_var_cast(var_cast)
    writer.run()
    return log_dir


def read_variable(log_dir, name):
    return pd.read_pickle(str(log_dir / f"{name}.data"))


def vector_run(values_per_step, n_steps=4, dt=1_000_000_000):
    """A retained dict for one run that contains one 'msg.var' message item."""
    times = np.arange(n_steps) * dt
    values = np.tile(np.asarray(values_per_step, dtype=float), (n_steps, 1))
    return {"messages": {"msg.var": make_item(times, values)}, "variables": {}, "custom": {}}


def test_constructing_a_writer_leaves_the_root_logger_alone(tmp_path):
    """The writer logs through its own named logger and does not change the root logger.

    Controller.load and JobRunner also make a writer in the parent process. If the writer
    adds a handler here, the output shows each message two times. The log level also
    increases for all other libraries in the process.
    """
    root = logging.getLogger()
    before_handlers = list(root.handlers)
    before_level = root.level

    DataWriter(queue.Queue())

    assert root.handlers == before_handlers
    assert root.level == before_level


def test_writes_one_file_per_retained_item(tmp_path):
    """The writer writes each item in each bucket to a different <name>.data file."""
    times = np.arange(3) * 1_000_000_000
    data = {
        "messages": {"attGuidMsg.sigma_BR": make_item(times, np.zeros((3, 3)))},
        "variables": {"bskSat.totOrbEnergy": make_item(times, np.zeros(3))},
        "custom": {"myCustom.value": make_item(times, np.zeros(3))},
    }
    run_writer(tmp_path, [(0, data)])

    written = sorted(p.name for p in tmp_path.glob("*.data"))
    assert written == [
        "attGuidMsg.sigma_BR.data",
        "bskSat.totOrbEnergy.data",
        "myCustom.value.data",
    ]


def test_three_component_roundtrip(tmp_path):
    """The finalized file is one DataFrame with a (runNum, varIdx) column index."""
    runs = [(i, vector_run([i, i + 0.5, i + 0.25])) for i in range(3)]
    run_writer(tmp_path, runs)

    df = read_variable(tmp_path, "msg.var")

    assert df.index.name == "time[ns]"
    assert df.columns.names == ["runNum", "varIdx"]
    assert df.columns.levshape == (3, 3)
    assert df.shape == (4, 9)
    np.testing.assert_allclose(df.loc[:, (2, 1)].to_numpy(), np.full(4, 2.5))


def test_scalar_item_keeps_one_component(tmp_path):
    """After a write and a read, a scalar payload (vari_len == 1) is one varIdx, not a Series."""
    times = np.arange(4) * 1_000_000_000
    data = {"messages": {}, "variables": {"sat.energy": make_item(times, np.arange(4.0))},
            "custom": {}}
    run_writer(tmp_path, [(0, data)])

    df = read_variable(tmp_path, "sat.energy")

    assert df.columns.levshape == (1, 1)
    np.testing.assert_allclose(df.loc[:, (0, 0)].to_numpy(), np.arange(4.0))


def test_case_colliding_name_is_renamed(tmp_path):
    """The writer gives OrbitalElements.Omega a different file name so that it cannot replace .omega on APFS."""
    times = np.arange(2) * 1_000_000_000
    data = {
        "messages": {},
        "variables": {
            "OrbitalElements.omega": make_item(times, np.zeros(2)),
            "OrbitalElements.Omega": make_item(times, np.ones(2)),
        },
        "custom": {},
    }
    run_writer(tmp_path, [(0, data)])

    written = sorted(p.name for p in tmp_path.glob("*.data"))
    assert written == ["OrbitalElements.Omega_Capital.data", "OrbitalElements.omega.data"]
    np.testing.assert_allclose(
        read_variable(tmp_path, "OrbitalElements.Omega_Capital").to_numpy().ravel(),
        np.ones(2),
    )


def test_lost_run_is_backfilled_with_nan_columns(tmp_path):
    """The frame also has columns for a run that sent no data. The reindex fills these columns with NaN."""
    runs = [(0, vector_run([1.0, 2.0, 3.0])), (2, vector_run([7.0, 8.0, 9.0]))]
    run_writer(tmp_path, runs)

    df = read_variable(tmp_path, "msg.var")

    assert df.columns.levshape == (3, 3)
    assert df.loc[:, 1].isna().all().all()
    np.testing.assert_allclose(df.loc[:, (2, 0)].to_numpy(), np.full(4, 7.0))


def test_leading_runs_lost_shifts_the_run_numbering(tmp_path):
    """The dense column index includes the lowest to the highest run index that the writer receives.

    Thus, a loss of run 0 does not give a new number to a run. Runs 3 and 4 of a five-run
    batch make a frame whose runNum level starts at 3 and has only two entries. The frame
    does not contain the three runs that the writer did not receive. It also does not show them
    as NaN. For this reason, levshape[0] does not always give the batch size.
    """
    runs = [(3, vector_run([1.0, 2.0, 3.0])), (4, vector_run([4.0, 5.0, 6.0]))]
    run_writer(tmp_path, runs)

    df = read_variable(tmp_path, "msg.var")

    assert df.columns.levshape == (2, 3)
    assert sorted(set(df.columns.get_level_values(0))) == [3, 4]


def test_degenerate_item_loses_its_data(tmp_path):
    """A non-array custom item becomes a 1x1 NaN frame. The writer discards the value."""
    data = {"messages": {}, "variables": {}, "custom": {"myFlag": "converged"}}
    run_writer(tmp_path, [(0, data)])

    df = read_variable(tmp_path, "myFlag")

    assert df.shape == (1, 1)
    assert df.isna().all().all()


def test_ragged_runs_produce_a_union_index(tmp_path):
    """For runs of different lengths, the time index is the union of the run time indexes. NaN fills the missing values."""
    runs = [(0, vector_run([1.0, 2.0, 3.0], n_steps=3)),
            (1, vector_run([4.0, 5.0, 6.0], n_steps=5))]
    run_writer(tmp_path, runs)

    df = read_variable(tmp_path, "msg.var")

    assert df.shape[0] == 5
    assert df.loc[:, (0, 0)].isna().sum() == 2
    assert df.loc[:, (1, 0)].notna().all()


def test_shorter_first_run_does_not_truncate_a_later_longer_run(tmp_path):
    """The arrival order does not set a limit on the time axis. A short run 0 does not set the maximum.

    This invariant is the most important for the acquisition scenarios. In these scenarios,
    the stop time changes with the dispersion, and run 0 can get the shortest case.
    """
    runs = [(0, vector_run([1.0, 2.0, 3.0], n_steps=2)),
            (1, vector_run([4.0, 5.0, 6.0], n_steps=6))]
    run_writer(tmp_path, runs)

    df = read_variable(tmp_path, "msg.var")

    assert df.shape[0] == 6
    np.testing.assert_allclose(df.loc[:, (1, 2)].to_numpy(), np.full(6, 6.0))


@pytest.mark.parametrize("var_cast", [None, "float", "integer"])
def test_var_cast_does_not_change_the_stored_dtype(tmp_path, var_cast):
    """The stored values are float64 for all values of var_cast.

    ``pd.to_numeric(..., downcast='float')`` returns a float32 Series. But the next statement,
    ``df.iloc[:, i] = var_comp``, writes these values into the float64 block of the frame. It
    does not replace the column. Thus the frame discards the float32 dtype before the data
    gets to the disk. The design intent is to decrease the on-disk footprint by half. At this
    time, this decrease does not occur.
    """
    run_writer(tmp_path, [(0, vector_run([1.0, 2.0, 3.0]))], var_cast=var_cast)

    df = read_variable(tmp_path, "msg.var")

    assert set(df.dtypes) == {np.dtype(np.float64)}


def test_unfinalized_file_reads_back_as_run_zero_in_a_bare_list(tmp_path):
    """Without the shutdown sentinel, the file contains one pickle for each run, not a frame.

    All three runs are on disk. But ``pd.read_pickle`` stops after the first pickle in the
    stream and returns the ``[df]`` list that holds that frame. Thus, if a writer stops
    before it finalizes the file, the file loads without an error. The file then gives only
    one run and no warning, and the run is not a DataFrame.
    """
    class NonBlockingQueue(queue.Queue):
        """Raise ``queue.Empty`` when the queue is empty, so that run() stops in the middle of the stream.

        This queue does not block. The file then has the same state as the file of a writer
        that stopped before it received the shutdown sentinel.
        """

        def get(self, block=True, timeout=None):
            return super().get(block=False)

    q = NonBlockingQueue()
    for run_index in range(3):
        q.put((vector_run([run_index, 0.0, 0.0]), run_index, None))

    writer = DataWriter(q)
    writer.set_log_dir(str(tmp_path))
    writer.set_var_cast(None)
    with pytest.raises(queue.Empty):
        writer.run()

    path = tmp_path / "msg.var.data"
    with open(path, "rb") as pkl:
        frames = []
        try:
            while True:
                frames.extend(pickle.load(pkl))
        except EOFError:
            pass

    assert len(frames) == 3

    recovered = pd.read_pickle(path)
    assert isinstance(recovered, list) and len(recovered) == 1
    assert list(recovered[0].columns.get_level_values(0)) == [0, 0, 0]
