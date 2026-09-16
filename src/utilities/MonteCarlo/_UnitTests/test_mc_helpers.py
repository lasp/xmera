# SPDX-License-Identifier: ISC
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
"""Pure-Python unit tests for MC helpers added during the refactor round."""

import numpy as np
import pytest

from xmera.utilities.MonteCarlo.Controller import Controller, FailureRecord
from xmera.utilities.MonteCarlo.Dispersions import (
    NormalDispersion,
    NormalVectorAngleDispersion,
    NormalVectorDispersion,
    OrbitalElementDispersion,
    SymmetricSolarArrayDispersion,
    SymmetricSolarArrayWithReferenceDispersion,
    UniformDispersion,
)
from xmera.utilities.MonteCarlo.RetentionPolicy import RetentionPolicy


# --- FailureRecord ------------------------------------------------------------

def test_failure_record_fields():
    rec = FailureRecord(run_index=5, exception_type="ValueError", traceback="<tb>")
    assert rec.run_index == 5
    assert rec.exception_type == "ValueError"
    assert rec.traceback == "<tb>"


def test_failure_record_defaults():
    rec = FailureRecord(run_index=0)
    assert rec.exception_type == ""
    assert rec.traceback == ""


def test_failure_record_is_immutable():
    rec = FailureRecord(run_index=1)
    with pytest.raises(Exception):
        rec.run_index = 2  # frozen dataclass


# --- Dispersion magnitude reuse ----------------------------------------------

def test_uniform_dispersion_magnitude_does_not_accumulate():
    disp = UniformDispersion("foo.bar", bounds=[-1.0, 1.0])
    for _ in range(5):
        disp.generate_string(sim=None)
    assert len(disp.magnitude) == 1, (
        f"Expected magnitude length 1 after reuse; got {len(disp.magnitude)} entries: {disp.magnitude}"
    )


def test_normal_dispersion_magnitude_does_not_accumulate():
    disp = NormalDispersion("foo.bar", mean=0.0, std_deviation=1.0)
    for _ in range(5):
        disp.generate_string(sim=None)
    assert len(disp.magnitude) == 1


# --- NormalVectorAngleDispersion bounds ---------------------------------------

class _VectorSim:
    """Stub with a nominal vector attribute, so that the tests can use vector-angle dispersions."""

    def __init__(self, vector_attr_name="vec", vector=(1.0, 0.0, 0.0)):
        setattr(self, vector_attr_name, np.asarray(vector, dtype=float))


def test_normal_vector_angle_dispersion_raises_when_bounds_unreachable():
    """If the bounds window is very small and the std is very large, the dispersion must raise at the resample limit."""
    np.random.seed(0)
    sim = _VectorSim("vec", (1.0, 0.0, 0.0))
    disp = NormalVectorAngleDispersion(
        var_name="vec",
        phi_std=10.0,            # very large spread
        theta_std=10.0,
        phi_bounds_off_nom=[-1e-9, 1e-9],   # near-zero window
        theta_bounds_off_nom=[-1e-9, 1e-9],
    )
    with pytest.raises(ValueError) as excinfo:
        disp.generate(sim)
    assert "could not sample within bounds" in str(excinfo.value)


def test_normal_vector_angle_dispersion_stays_within_bounds():
    """If the std is small compared to the bounds, each sample must stay in the bounds."""
    np.random.seed(0)
    sim = _VectorSim("vec", (1.0, 0.0, 0.0))
    disp = NormalVectorAngleDispersion(
        var_name="vec",
        phi_std=0.05,
        theta_std=0.05,
        phi_bounds_off_nom=[-0.5, 0.5],
        theta_bounds_off_nom=[-0.5, 0.5],
    )
    # The executor calls generate_string for each run, and generate_string sets the magnitude
    # list to empty. Thus this test uses generate_string and not generate.
    for _ in range(50):
        value = disp.generate_string(sim)
        assert value.startswith("[") and value.endswith("]")
        components = [float(part) for part in value[1:-1].split(",")]
        assert len(components) == 3
        # One pair for each run. The list is empty at the start of each run, so its length does not increase.
        assert len(disp.magnitude) == 2


# --- RetentionPolicy duplicate handling ---------------------------------------

class _SimStubForRetention:
    def __init__(self):
        self.added = []

    def AddVariableForMultiProcessLogging(self, name, rate, start, stop, var_type):
        self.added.append((name, rate, start, stop, var_type))


def test_retention_policy_drops_duplicate_variable_across_policies(caplog):
    sim = _SimStubForRetention()
    policy_a = RetentionPolicy(rate=int(1e10))
    policy_a.add_variable_log("module.foo")
    policy_b = RetentionPolicy(rate=int(1e10))
    policy_b.add_variable_log("module.foo")  # same entry as policy_a

    with caplog.at_level("WARNING", logger="montecarlo_retention"):
        RetentionPolicy.add_retention_policies_to_sim(sim, [policy_a, policy_b])

    assert len(sim.added) == 1, f"Expected one AddVariable call, got {sim.added}"
    assert sim.added[0][0] == "module.foo"
    assert any("Duplicate retention entry" in rec.message for rec in caplog.records), (
        "Expected a WARNING-level log when dropping the duplicate"
    )


def test_retention_policy_drops_duplicate_variable_within_one_policy():
    sim = _SimStubForRetention()
    policy = RetentionPolicy(rate=int(1e10))
    policy.add_variable_log("module.foo")
    policy.add_variable_log("module.foo")
    RetentionPolicy.add_retention_policies_to_sim(sim, [policy])
    assert len(sim.added) == 1


def test_retention_policy_keeps_distinct_variables():
    sim = _SimStubForRetention()
    policy = RetentionPolicy(rate=int(1e10))
    policy.add_variable_log("module.foo")
    policy.add_variable_log("module.bar")
    RetentionPolicy.add_retention_policies_to_sim(sim, [policy])
    assert {entry[0] for entry in sim.added} == {"module.foo", "module.bar"}


# --- NormalVectorDispersion ---------------------------------------------------

class _VectorSimStub:
    vector = [0.0, 0.0, 0.0]


def test_normal_vector_dispersion_uses_its_mean_and_deviation():
    dispersion = NormalVectorDispersion("vector", mean=2.0, std_deviation=0.0)
    np.testing.assert_allclose(dispersion.generate(_VectorSimStub()), [2.0, 2.0, 2.0])


# --- OrbitalElementDispersion indexes -----------------------------------------

def test_orbital_element_dispersion_rejects_an_unknown_index():
    dispersion = OrbitalElementDispersion("position", "velocity", {"mu": 1.0})
    with pytest.raises(IndexError):
        dispersion.get_name(3)
    with pytest.raises(IndexError):
        dispersion.generate_string(3)


# --- Controller log level -----------------------------------------------------

@pytest.mark.parametrize("level", ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"])
def test_log_level_accepts_logging_level_names(level):
    monte_carlo = Controller()
    monte_carlo.log_level = level
    assert monte_carlo.log_level == level


@pytest.mark.parametrize("level", ["info", "NOT_A_LEVEL", ""])
def test_log_level_rejects_other_names(level):
    with pytest.raises(ValueError):
        Controller().log_level = level


# --- Solar array dispersion indexes -------------------------------------------

@pytest.mark.parametrize("dispersion", [
    SymmetricSolarArrayDispersion(*["angle"] * 6, bounds=[0.0, 1.0]),
    SymmetricSolarArrayWithReferenceDispersion(*["angle"] * 8, bounds=[0.0, 1.0]),
], ids=["symmetric", "with_reference"])
def test_solar_array_dispersion_rejects_an_unknown_index(dispersion):
    dispersion.generate()
    with pytest.raises(IndexError):
        dispersion.get_name(0)
    with pytest.raises(IndexError):
        dispersion.generate_string(dispersion.number_of_sub_disps + 1)
