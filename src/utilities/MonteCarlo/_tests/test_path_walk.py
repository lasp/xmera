# SPDX-License-Identifier: ISC
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
"""Unit tests for the Monte Carlo attribute path walk."""

import pytest

from xmera.utilities.MonteCarlo.PathWalk import (
    DispersionApplyError,
    _apply_modification,
    _tokenize_path,
)


class _SimStub:
    """Object with the same shape as a sim instance, for tests of the attribute walk."""

    def __init__(self):
        self.scalar = 0.0
        self.nested = _Nested()
        self.values = [_Nested(), _Nested(), _Nested()]
        self._calls = []

    def method(self, x):
        self._calls.append(("method", x))


class _Nested:
    def __init__(self):
        self.attr = 0.0
        self.flag = False


# --- _tokenize_path ----------------------------------------------------------

def test_tokenize_path_attr_chain():
    assert _tokenize_path("a.b.c") == [("attr", "a"), ("attr", "b"), ("attr", "c")]


def test_tokenize_path_indexed():
    assert _tokenize_path("a[2].b[10].c") == [
        ("attr", "a"), ("index", 2), ("attr", "b"), ("index", 10), ("attr", "c"),
    ]


def test_tokenize_path_empty_raises():
    with pytest.raises(DispersionApplyError):
        _tokenize_path("")


def test_tokenize_path_seed_path():
    assert _tokenize_path("TaskList[0].TaskModels[3].RNGSeed") == [
        ("attr", "TaskList"), ("index", 0), ("attr", "TaskModels"), ("index", 3), ("attr", "RNGSeed")
    ]


@pytest.mark.parametrize("path", [
    "get_model().x",
    "a.b()",
    "a..b",
    ".a",
    "a.",
    "a b",
    "a[0]b",
    "[0].a",
    "a[idx]",
    "a-b",
])
def test_tokenize_path_rejects_unparseable_syntax(path):
    with pytest.raises(DispersionApplyError):
        _tokenize_path(path)


# --- _apply_modification ------------------------------------------------------

def test_apply_modification_scalar_attr():
    sim = _SimStub()
    _apply_modification(sim, "scalar", "3.14")
    assert sim.scalar == pytest.approx(3.14)


def test_apply_modification_nested_attr():
    sim = _SimStub()
    _apply_modification(sim, "nested.attr", "42")
    assert sim.nested.attr == 42


def test_apply_modification_indexed():
    sim = _SimStub()
    _apply_modification(sim, "values[1].attr", "7.5")
    assert sim.values[1].attr == pytest.approx(7.5)


def test_apply_modification_bool_literal():
    sim = _SimStub()
    _apply_modification(sim, "nested.flag", "True")
    assert sim.nested.flag is True


def test_apply_modification_callable_with_scalar():
    sim = _SimStub()
    _apply_modification(sim, "method", "5")
    assert sim._calls == [("method", 5)]


def test_apply_modification_unknown_attr_raises():
    sim = _SimStub()
    with pytest.raises(DispersionApplyError) as excinfo:
        _apply_modification(sim, "nested.missing", "1.0")
    assert "missing" in str(excinfo.value) or "leaf" in str(excinfo.value)


def test_apply_modification_unparseable_value_raises():
    sim = _SimStub()
    with pytest.raises(DispersionApplyError) as excinfo:
        _apply_modification(sim, "scalar", "np.array([1])")
    assert "parse value" in str(excinfo.value)


def test_apply_modification_unknown_top_level_attr_raises():
    sim = _SimStub()
    with pytest.raises(DispersionApplyError):
        _apply_modification(sim, "nonexistent.attr", "1.0")
