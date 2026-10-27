# SPDX-License-Identifier: ISC
# Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
"""A Monte Carlo archive of preallocated NumPy arrays, one directory per retained item.

Each item ``<name>`` (for example ``attGuidMsg.sigma_BR``) has the directory ``<results>/<name>/`` with:

- ``values.npy``: the values, shape ``(rows, components, samples)``. A run is one row. Each run is written
  into its row when it arrives, so the archive needs no concatenation at the end of the batch and a reader
  can memory map one item without reading the others.
- ``time_ns.npy``: the sample times [ns], shared by all runs.
- ``runs.npy``: the run index of each row, ``-1`` for a row that has no run yet.
- ``meta.json``: the retained-data bucket of the item (``messages``, ``variables`` or ``custom``).

Normally run ``i`` is in row ``i``. A sample that a run does not have is NaN. Use
``Controller.set_archive_format("npy")`` to write this archive instead of the pickled DataFrames of
:class:`DataWriter`.
"""

import json
import logging
import multiprocessing as mp
import os

import numpy as np

logger = logging.getLogger("montecarlo_arrayarchive")

VALUES, TIMES, RUNS, META = "values.npy", "time_ns.npy", "runs.npy", "meta.json"


def item_directory(results_dir, name):
    return os.path.join(results_dir, name)


def is_item(path):
    return os.path.isfile(os.path.join(path, VALUES))


def item_names(results_dir):
    """The names of the items in an archive."""
    if not os.path.isdir(results_dir):
        return []
    return sorted(name for name in os.listdir(results_dir) if is_item(os.path.join(results_dir, name)))


def _save_atomic(path, array):
    temporary = path + ".tmp.npy"
    np.save(temporary, array)
    os.replace(temporary, path)


class Item:
    """One item of an archive, opened for writing runs into it."""

    def __init__(self, directory):
        self.directory = directory
        self.values = np.load(os.path.join(directory, VALUES), mmap_mode="r+")
        self.time_ns = np.load(os.path.join(directory, TIMES))
        self.runs = np.load(os.path.join(directory, RUNS))

    @classmethod
    def create(cls, directory, bucket, time_ns, components, dtype, rows):
        os.makedirs(directory, exist_ok=True)
        np.lib.format.open_memmap(
            os.path.join(directory, VALUES), mode="w+", dtype=dtype, shape=(rows, components, len(time_ns))
        ).flush()
        np.save(os.path.join(directory, TIMES), np.asarray(time_ns, dtype=np.int64))
        _save_atomic(os.path.join(directory, RUNS), np.full(rows, -1, dtype=np.int64))
        with open(os.path.join(directory, META), "w") as meta:
            json.dump({"bucket": bucket}, meta)
        return cls(directory)

    def _row(self, run_index):
        """The row of a run: its own row if it is there already, else row run_index if free, else the first free
        row. The archive grows when it has no free row."""
        existing = np.flatnonzero(self.runs == run_index)
        if existing.size:
            return int(existing[0])
        if run_index < len(self.runs) and self.runs[run_index] == -1:
            return run_index
        free = np.flatnonzero(self.runs == -1)
        if free.size:
            return int(free[0])
        self._resize(rows=max(run_index + 1, 2 * len(self.runs)), time_ns=self.time_ns)
        return self._row(run_index)

    def _resize(self, rows, time_ns):
        """Copy the item into a file with more rows or on a larger time grid, one row at a time."""
        old_values, old_time, old_runs = self.values, self.time_ns, self.runs
        position = np.searchsorted(time_ns, old_time)
        path = os.path.join(self.directory, VALUES)
        temporary = path + ".resize.npy"
        values = np.lib.format.open_memmap(
            temporary, mode="w+", dtype=old_values.dtype, shape=(rows, old_values.shape[1], len(time_ns))
        )
        for row in np.flatnonzero(old_runs >= 0):
            values[row] = np.nan
            values[row][:, position] = old_values[row]
        values.flush()
        del values, old_values
        self.values = None
        os.replace(temporary, path)
        np.save(os.path.join(self.directory, TIMES), np.asarray(time_ns, dtype=np.int64))
        runs = np.full(rows, -1, dtype=np.int64)
        runs[: len(old_runs)] = old_runs
        _save_atomic(os.path.join(self.directory, RUNS), runs)
        self.values = np.load(path, mmap_mode="r+")
        self.time_ns, self.runs = np.asarray(time_ns, dtype=np.int64), runs

    def write(self, run_index, time_ns, values):
        """Write one run, values of shape (samples, components) at time_ns. A run on other sample times than the
        item's first run widens the item's time grid."""
        time_ns = np.asarray(time_ns, dtype=np.int64)
        if not np.array_equal(time_ns, self.time_ns):
            grid = np.union1d(self.time_ns, time_ns)
            if len(grid) != len(self.time_ns):
                logger.info(f"Run {run_index} widens the time grid of {self.directory}")
                self._resize(len(self.runs), grid)
        if values.shape[1] > self.values.shape[1]:
            logger.warning(f"Run {run_index} of {self.directory} has more components than the first run; "
                           f"dropping components {self.values.shape[1]} and up")
        row = self._row(run_index)
        components = min(values.shape[1], self.values.shape[1])
        target = self.values[row]
        if np.array_equal(time_ns, self.time_ns):
            target[:components] = values[:, :components].T
            target[components:] = np.nan
        else:
            target[:] = np.nan
            target[:components, np.searchsorted(self.time_ns, time_ns)] = values[:, :components].T
        self.values.flush()
        # The run index goes in last, so an interrupted write leaves the row free
        self.runs[row] = run_index
        _save_atomic(os.path.join(self.directory, RUNS), self.runs)


def numeric_values(item_data):
    """(time_ns, values (samples, components)) of a retained-data array, or None when it is not a numeric
    time series (a scalar, a missing item, or a payload of structures)."""
    try:
        array = np.asarray(item_data)
        if array.ndim != 2 or array.shape[1] < 2:
            return None
        return array[:, 0].astype(np.int64), np.asarray(array[:, 1:], dtype=float)
    except (TypeError, ValueError):
        return None


def write_run(results_dir, data, run_index, dtype=np.float64, rows=1, items=None, renames=None, skipped=None):
    """Write one run's retained data, {bucket: {name: array}}, into the archive. items caches the open
    items between calls, and skipped the names already reported as not archived; rows is the number of rows a
    new item starts with."""
    items = {} if items is None else items
    skipped = set() if skipped is None else skipped
    for bucket, bucket_data in data.items():
        if not isinstance(bucket_data, dict):
            continue
        for name, item_data in bucket_data.items():
            name = (renames or {}).get(name, name)
            series = numeric_values(item_data)
            if series is None:
                if name not in skipped:
                    logger.warning(f"Not archiving {name}: it is not a numeric time series")
                    skipped.add(name)
                continue
            time_ns, values = series
            if name not in items:
                directory = item_directory(results_dir, name)
                items[name] = (
                    Item(directory) if is_item(directory)
                    else Item.create(directory, bucket, time_ns, values.shape[1], dtype, max(rows, run_index + 1))
                )
            items[name].write(run_index, time_ns, values.astype(dtype, copy=False))
    return items


def read(results_dir, name):
    """(time_ns, run indexes, values) of one item, rows in run order. values is read only and memory mapped,
    shape (runs, components, samples); it is a copy only when the rows are not already in run order."""
    directory = item_directory(results_dir, name)
    values = np.load(os.path.join(directory, VALUES), mmap_mode="r")
    time_ns = np.load(os.path.join(directory, TIMES))
    runs = np.load(os.path.join(directory, RUNS))
    rows = np.flatnonzero(runs >= 0)
    rows = rows[np.argsort(runs[rows], kind="stable")]
    if np.array_equal(rows, np.arange(len(rows))):
        return time_ns, runs[rows], values[: len(rows)]
    return time_ns, runs[rows], values[rows]


def runs_in(results_dir, name):
    """The run indexes that one item holds."""
    runs = np.load(os.path.join(item_directory(results_dir, name), RUNS))
    return {int(run) for run in runs if run >= 0}


def retained_data(results_dir, run_index):
    """One run's retained data, {bucket: {name: array}} with the time [ns] in column 0, as the retention policy
    gave it to the writer (for data callbacks when the batch keeps no run<index>.data files)."""
    data = {"messages": {}, "variables": {}, "custom": {}}
    for name in item_names(results_dir):
        directory = item_directory(results_dir, name)
        runs = np.load(os.path.join(directory, RUNS))
        rows = np.flatnonzero(runs == run_index)
        if not rows.size:
            continue
        with open(os.path.join(directory, META)) as meta:
            bucket = json.load(meta)["bucket"]
        values = np.load(os.path.join(directory, VALUES), mmap_mode="r")[rows[0]]
        time_ns = np.load(os.path.join(directory, TIMES))
        data.setdefault(bucket, {})[name] = np.column_stack([time_ns.astype(float), np.asarray(values, float).T])
    data["index"] = run_index
    return data


class ArrayDataWriter(mp.Process):
    """The data writer of the ``npy`` archive format: it takes each run from the queue and writes it into its
    row of each item's arrays. Use it like :class:`DataWriter`."""

    def __init__(self, q, rows=1):
        super().__init__()
        self._queue = q
        self._end_token = None
        self._log_dir = ""
        self._dtype = np.float64
        self._rows = rows

    def run(self):
        from .DataWriter import DataWriter

        items, skipped = {}, set()
        while self._end_token is None:
            data, run_index, self._end_token = self._queue.get()
            if self._end_token:
                continue
            logger.info(f"Writing run {run_index}")
            write_run(self._log_dir, data, run_index, self._dtype, self._rows, items,
                      DataWriter._CASE_INSENSITIVE_RENAMES, skipped)

    def set_log_dir(self, log_dir):
        self._log_dir = log_dir

    def set_var_cast(self, var_cast):
        """'float' stores float32, as DataWriter's downcast intends; anything else stores float64."""
        self._dtype = np.float32 if var_cast == "float" else np.float64
