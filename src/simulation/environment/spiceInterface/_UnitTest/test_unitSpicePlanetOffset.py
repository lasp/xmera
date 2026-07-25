# SPDX-License-Identifier: ISC
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import inspect
import os

import numpy as np
import numpy.testing as npt
import pytest

# Exercises the SecondaryBody capability of SpiceInterface:
#   - the secondary state output message equals the primary's state plus
#     the configured offset, and the body is renamed
#   - the primary planet message itself is untouched
#   - when an orbital period is set, the secondary traces a circle of radius |offset|
#     centered on the primary, in the equator-tilted plane normal to
#     offset x (offset x J2000 Z)

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
from xmera import __path__
bsk_path = __path__[0]

from xmera.utilities import SimulationBaseClass
from xmera.simulation import spiceInterface
from xmera.utilities import macros
from xmera.architecture import messaging  # noqa: F401  registers Message<X> Python wrappers


planet_names = ["earth", "moon"]
primary_name = "moon"
primary_index = 1
secondary_name = "binaryAsteroid"
default_offset = np.array([1000.0, 2000.0, -3000.0])  # m
date_spice = "2015 February 10, 00:00:00.0 TDB"
# Position-difference tolerance: secondary - primary involves subtracting two
# SPICE-derived positions of order 1e8 m, so the float-precision floor is ~1e-8 m.
# 1e-3 m (1 mm) is well below any physically meaningful threshold.
position_atol = 1e-3  # m


def build_sim(orbital_period, stop_time_sec, time_step_sec=0.1, attach_secondary=True,
              offset=default_offset):
    """Configure a SpiceInterface, attach recorders, run the sim, and return
    (spice, secondary_rec, primary_rec)."""
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("TestProcess")
    proc.addTask("unitTask", macros.sec2nano(time_step_sec))

    spice = spiceInterface.SpiceInterface()
    spice.modelTag = "SpiceInterface"
    spice.SPICEDataPath = bsk_path + "/supportData/EphemerisData/"
    spice.addPlanetNames(planet_names)
    spice.UTCCalInit = date_spice

    if attach_secondary:
        secondary = spiceInterface.SecondaryBody()
        secondary.secondaryName = secondary_name
        secondary.positionOffset = np.asarray(offset, dtype=float)
        secondary.orbitalPeriod = orbital_period
        spice.setOffsetBody(primary_name, secondary)

    secondary_rec = spice.secondaryStateOutMsg.recorder()
    primary_rec = spice.planetStateOutMsgs[primary_index].recorder()

    sim.AddModelToTask("unitTask", spice)
    sim.AddModelToTask("unitTask", secondary_rec)
    sim.AddModelToTask("unitTask", primary_rec)
    sim.ConfigureStopTime(macros.sec2nano(stop_time_sec))
    sim.InitializeSimulation()
    sim.ExecuteSimulation()

    return spice, secondary_rec, primary_rec


def test_secondary_body_static_offset(show_plots):
    """Period=0: secondary position = primary position + constant offset; name is renamed."""
    spice, secondary_rec, primary_rec = build_sim(orbital_period=0.0, stop_time_sec=0.5)

    delta = np.array(secondary_rec.PositionVector) - np.array(primary_rec.PositionVector)
    npt.assert_allclose(delta, np.broadcast_to(default_offset, delta.shape), atol=position_atol,
                        err_msg="static offset mismatch in secondary message")

    name = spice.secondaryStateOutMsg.read().PlanetName
    npt.assert_equal(name, secondary_name)


def test_primary_body_unchanged(show_plots):
    """Attaching a SecondaryBody must not perturb the primary's planet message."""
    _, _, primary_with_rec = build_sim(orbital_period=0.0, stop_time_sec=0.5,
                                       attach_secondary=True)
    _, _, primary_ref_rec = build_sim(orbital_period=0.0, stop_time_sec=0.5,
                                      attach_secondary=False)

    npt.assert_allclose(np.array(primary_with_rec.PositionVector),
                        np.array(primary_ref_rec.PositionVector),
                        atol=position_atol,
                        err_msg="primary planet message changed when SecondaryBody attached")


def _expected_orbit_basis(offset):
    """Replicates the C++ orbit-plane construction: plane_normal = offset x (offset x up),
    which tilts the orbit toward the equator rather than through the poles. Falls back
    to the J2000 X axis when offset is parallel to up."""
    parallel_tol = 8.0 * np.finfo(np.float64).eps
    up = np.array([0.0, 0.0, 1.0])
    intermediate = np.cross(offset, up)
    if np.linalg.norm(intermediate) < parallel_tol * np.linalg.norm(offset):
        intermediate = np.cross(offset, np.array([1.0, 0.0, 0.0]))
    plane_normal = np.cross(offset, intermediate)
    plane_normal /= np.linalg.norm(plane_normal)
    return plane_normal, np.cross(plane_normal, offset)


# Offsets cover: generic 3-component, pure +X, pure +Y, asymmetric in-plane,
# and the parallel-to-up degenerate case (forces the X-axis fallback in the
# C++ basis construction).
orbit_offsets = [
    pytest.param(np.array([1000.0, 2000.0, -3000.0]), id="mixed"),
    pytest.param(np.array([5000.0, 0.0, 0.0]),        id="pure_x"),
    pytest.param(np.array([0.0, 4000.0, 0.0]),        id="pure_y"),
    pytest.param(np.array([-500.0, 1500.0, 0.0]),     id="asymmetric_xy"),
    pytest.param(np.array([0.0, 0.0, 2500.0]),        id="parallel_to_up"),
]


@pytest.mark.parametrize("offset", orbit_offsets)
def test_secondary_body_circular_orbit(show_plots, offset):
    """Period=T: secondary orbits the primary in the equator-tilted plane normal to
    offset x (offset x J2000 Z) (or offset x (offset x X) when offset is parallel to Z),
    and returns to start at t=T."""
    period = 10.0  # s
    time_step = period / 20.0
    _, secondary_rec, primary_rec = build_sim(
        orbital_period=period, stop_time_sec=period, time_step_sec=time_step,
        offset=offset,
    )

    times = np.array(secondary_rec.times()) * macros.NANO2SEC
    delta = np.array(secondary_rec.PositionVector) - np.array(primary_rec.PositionVector)

    angles = 2.0 * np.pi * times / period
    plane_normal, perp = _expected_orbit_basis(offset)
    expected = (np.outer(np.cos(angles), offset)
                + np.outer(np.sin(angles), perp))
    npt.assert_allclose(delta, expected, atol=position_atol,
                        err_msg="circular-orbit rotation mismatch")

    # Radius is conserved.
    r_3d = np.linalg.norm(delta, axis=1)
    npt.assert_allclose(r_3d, np.linalg.norm(offset), atol=position_atol,
                        err_msg="orbital radius drifted")

    # Trajectory stays in the plane (projection onto the plane normal is zero).
    out_of_plane = delta @ plane_normal
    npt.assert_allclose(out_of_plane, 0.0, atol=position_atol,
                        err_msg="secondary drifted out of orbital plane")

    # Closes the loop: t=0 and t=T positions match.
    npt.assert_allclose(delta[-1], delta[0], atol=position_atol,
                        err_msg="orbit did not close after one period")

    if show_plots:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        fig = plt.figure(figsize=(7, 6))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(delta[:, 0], delta[:, 1], delta[:, 2], "o-",
                label="secondary - primary")
        ax.scatter([offset[0]], [offset[1]], [offset[2]],
                   color="r", s=80, label="initial offset")
        ax.scatter([0], [0], [0], color="k", s=40, label="primary")
        ax.set_xlabel("X offset [m]")
        ax.set_ylabel("Y offset [m]")
        ax.set_zlabel("Z offset [m]")
        ax.set_title(f"Secondary body trajectory (T={period}s, offset={offset.tolist()})")
        ax.legend()
        plt.show()


if __name__ == "__main__":
    test_secondary_body_static_offset(True)
    test_primary_body_unchanged(True)
    for params in orbit_offsets:
        test_secondary_body_circular_orbit(True, params.values[0])
    print(" \n PASSED ")
