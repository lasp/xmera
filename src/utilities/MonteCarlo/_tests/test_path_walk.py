# SPDX-License-Identifier: ISC
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
"""Unit tests for the Monte Carlo attribute path walk."""

import pytest

from xmera.utilities.MonteCarlo.PathWalk import (
    DispersionApplyError,
    _tokenize_path,
    apply_dispersions,
    apply_modification,
    generate_modifications,
    resolve_path,
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


# --- apply_modification -------------------------------------------------------

def test_apply_modification_scalar_attr():
    sim = _SimStub()
    apply_modification(sim, "scalar", "3.14")
    assert sim.scalar == pytest.approx(3.14)


def test_apply_modification_nested_attr():
    sim = _SimStub()
    apply_modification(sim, "nested.attr", "42")
    assert sim.nested.attr == 42


def test_apply_modification_indexed():
    sim = _SimStub()
    apply_modification(sim, "values[1].attr", "7.5")
    assert sim.values[1].attr == pytest.approx(7.5)


def test_apply_modification_bool_literal():
    sim = _SimStub()
    apply_modification(sim, "nested.flag", "True")
    assert sim.nested.flag is True


def test_apply_modification_callable_with_scalar():
    sim = _SimStub()
    apply_modification(sim, "method", "5")
    assert sim._calls == [("method", 5)]


def test_apply_modification_unknown_attr_raises():
    sim = _SimStub()
    with pytest.raises(DispersionApplyError) as excinfo:
        apply_modification(sim, "nested.missing", "1.0")
    assert "missing" in str(excinfo.value) or "leaf" in str(excinfo.value)


def test_apply_modification_unparseable_value_raises():
    sim = _SimStub()
    with pytest.raises(DispersionApplyError) as excinfo:
        apply_modification(sim, "scalar", "np.array([1])")
    assert "parse value" in str(excinfo.value)


def test_apply_modification_unknown_top_level_attr_raises():
    sim = _SimStub()
    with pytest.raises(DispersionApplyError):
        apply_modification(sim, "nonexistent.attr", "1.0")


# --- resolve_path -------------------------------------------------------------

def test_resolve_path_returns_leaf():
    sim = _SimStub()
    sim.values[2].attr = 9.0
    assert resolve_path(sim, "values[2].attr") == 9.0


def test_resolve_path_unknown_segment_raises():
    with pytest.raises(DispersionApplyError):
        resolve_path(_SimStub(), "nested.missing.attr")


# --- generate_modifications and apply_dispersions -----------------------------

class _SingleDispersion:
    def __init__(self, name, value):
        self._name = name
        self._value = value

    def get_name(self):
        return self._name

    def generate_string(self, sim=None):
        return repr(self._value)

    def generate_mag_string(self):
        return "1 sigma"


class _CoDependentDispersion:
    """Two values that one ``generate`` call makes together, like an orbit position and velocity."""

    number_of_sub_disps = 2

    def __init__(self):
        self.generate_calls = 0

    def generate(self, sim=None):
        self.generate_calls += 1

    def get_name(self, index):
        return {1: "scalar", 2: "nested.attr"}[index]

    def generate_string(self, index, sim=None):
        return {1: "1.5", 2: "2.5"}[index]

    def generate_mag_string(self):
        return "co-dependent"


def test_generate_modifications_single_and_co_dependent():
    co_dependent = _CoDependentDispersion()
    modifications = generate_modifications(
        _SimStub(), [_SingleDispersion("values[0].attr", 3.0), co_dependent]
    )
    assert modifications == {"values[0].attr": "3.0", "scalar": "1.5", "nested.attr": "2.5"}
    assert co_dependent.generate_calls == 1


def test_generate_modifications_keeps_saved_values():
    saved = {"values[0].attr": "8.0"}
    modifications = generate_modifications(_SimStub(), [_SingleDispersion("values[0].attr", 3.0)], saved)
    assert modifications is saved
    assert modifications == {"values[0].attr": "8.0"}


def test_generate_modifications_records_magnitudes():
    magnitudes = {}
    generate_modifications(
        _SimStub(), [_SingleDispersion("scalar", 1.0), _CoDependentDispersion()], magnitudes=magnitudes
    )
    assert magnitudes == {"scalar": "1 sigma", "nested.attr": "co-dependent"}


def test_apply_dispersions_sets_values_and_calls_setters():
    sim = _SimStub()
    applied = apply_dispersions(sim, [_SingleDispersion("values[1].attr", 4.0), _SingleDispersion("method", 6)])
    assert applied == {"values[1].attr": "4.0", "method": "6"}
    assert sim.values[1].attr == 4.0
    assert sim._calls == [("method", 6)]


def test_apply_dispersions_raises_on_bad_path():
    with pytest.raises(DispersionApplyError):
        apply_dispersions(_SimStub(), [_SingleDispersion("get_model().scalar", 1.0)])
