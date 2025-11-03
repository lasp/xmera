# ISC License
#
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

from xmera.utilities import orbitalMotion as oe_py
from xmera.architecture.orbitalMotion import OrbitalMotion as oe_cpp
from xmera.architecture.orbitalMotion import ClassicalElements as elements_cpp
import numpy as np


def test_rv_oe():
    gravitational_parameter = 1e10
    position = np.array([1e5, 1e6, 1e2])
    velocity = np.array([10, 20, 30])

    elements = oe_py.rv2elem(gravitational_parameter, position, velocity)
    r, v = oe_py.elem2rv(gravitational_parameter, elements)
    np.testing.assert_almost_equal(position, r, 8)
    np.testing.assert_almost_equal(velocity, v, 8)

    gravitational_parameter = 1e10
    position = np.array([1e5, 1e6, 0])
    h = np.array([0.0, 0.0, 1.0])
    velocity = np.sqrt(gravitational_parameter / np.linalg.norm(position)) * np.cross(
        h, position / np.linalg.norm(position)
    )
    elements = oe_py.rv2elem(gravitational_parameter, position, velocity)
    np.testing.assert_almost_equal(elements.a, np.linalg.norm(position), 8)
    np.testing.assert_almost_equal(elements.e, 0, 8)
    np.testing.assert_almost_equal(elements.i, 0, 8)


def test_compare_conversions():
    gravitational_parameter = 1e10
    position = np.array([1e5, 1e6, 1e2])
    velocity = np.array([10, 20, 30])

    # state vector to orbital elements
    elements_py = oe_py.rv2elem(gravitational_parameter, position, velocity)
    elements_cpp = oe_cpp.cartesianStateToElements(gravitational_parameter, position, velocity)
    np.testing.assert_almost_equal(elements_py.a, elements_cpp.semiMajorAxis, 8)
    np.testing.assert_almost_equal(elements_py.e, elements_cpp.eccentricity, 8)
    np.testing.assert_almost_equal(elements_py.i, elements_cpp.inclination, 8)
    np.testing.assert_almost_equal(elements_py.Omega, elements_cpp.rightAscensionAscendingNode, 8)
    np.testing.assert_almost_equal(elements_py.omega, elements_cpp.argPeriapsis, 8)
    np.testing.assert_almost_equal(elements_py.f, elements_cpp.trueAnomaly, 8)

    # orbital elements to state vectors
    r_py, v_py = oe_py.elem2rv(gravitational_parameter, elements_py)
    state_cpp = oe_cpp.elementsToCartesianState(gravitational_parameter, elements_cpp)
    np.testing.assert_almost_equal(r_py, np.array(state_cpp.position).flatten(), 8)
    np.testing.assert_almost_equal(v_py, np.array(state_cpp.velocity).flatten(), 8)

    eccentric_anomaly = 0.5
    # eccentric to true anomaly and back
    true_anomaly_py = oe_py.E2f(eccentric_anomaly, elements_py.e)
    true_anomaly_cpp = oe_cpp.eccentricToTrueAnomaly(eccentric_anomaly, elements_py.e)
    np.testing.assert_almost_equal(true_anomaly_py, true_anomaly_cpp, 8)

    eccentric_anomaly_py = oe_py.f2E(true_anomaly_py, elements_py.e)
    eccentric_anomaly_cpp = oe_cpp.trueToEccentricAnomaly(true_anomaly_cpp, elements_py.e)
    np.testing.assert_almost_equal(eccentric_anomaly, eccentric_anomaly_py, 8)
    np.testing.assert_almost_equal(eccentric_anomaly, eccentric_anomaly_cpp, 8)

    # eccentric to mean anomaly
    mean_anomaly_py = oe_py.E2M(eccentric_anomaly, elements_py.e)
    mean_anomaly_cpp = oe_cpp.eccentricToMeanAnomaly(eccentric_anomaly, elements_py.e)
    np.testing.assert_almost_equal(mean_anomaly_py, mean_anomaly_cpp, 8)

    eccentric_anomaly_py = oe_py.M2E(mean_anomaly_py, elements_py.e)
    eccentric_anomaly_cpp = oe_cpp.meanToEccentricAnomaly(mean_anomaly_cpp, elements_py.e)
    np.testing.assert_almost_equal(eccentric_anomaly_py, eccentric_anomaly_cpp, 8)
    np.testing.assert_almost_equal(eccentric_anomaly_py, eccentric_anomaly, 8)
    np.testing.assert_almost_equal(eccentric_anomaly_cpp, eccentric_anomaly, 8)

    # true to hyperbolic anomaly
    e_hyperbolic = 1.2
    true_anomaly = 0.234
    hyper_anomaly_py = oe_py.f2H(true_anomaly, e_hyperbolic)
    hyper_anomaly_cpp = oe_cpp.trueToHyperbolicAnomaly(true_anomaly, e_hyperbolic)
    np.testing.assert_almost_equal(hyper_anomaly_py, hyper_anomaly_cpp, 8)

    true_anomaly_py = oe_py.H2f(hyper_anomaly_py, e_hyperbolic)
    true_anomaly_cpp = oe_cpp.hyperbolicToTrueAnomaly(hyper_anomaly_cpp, e_hyperbolic)
    np.testing.assert_almost_equal(true_anomaly_py, true_anomaly_cpp, 8)
    np.testing.assert_almost_equal(true_anomaly_py, true_anomaly, 8)
    np.testing.assert_almost_equal(true_anomaly_cpp, true_anomaly, 8)

    # true to mean anomaly
    true_anomaly = 0.234
    mean_anomaly_py = oe_py.E2M(oe_py.f2E(true_anomaly, elements_py.e), elements_py.e)
    mean_anomaly_cpp = oe_cpp.trueToMeanAnomaly(true_anomaly, elements_py.e)
    np.testing.assert_almost_equal(mean_anomaly_py, mean_anomaly_cpp, 8)

    true_anomaly_py = oe_py.E2f(oe_py.M2E(mean_anomaly_py, elements_py.e), elements_py.e)
    true_anomaly_cpp = oe_cpp.meanToTrueAnomaly(mean_anomaly_cpp, elements_py.e)
    np.testing.assert_almost_equal(true_anomaly_py, true_anomaly_cpp, 8)
    np.testing.assert_almost_equal(true_anomaly_py, true_anomaly, 8)
    np.testing.assert_almost_equal(true_anomaly_cpp, true_anomaly, 8)
