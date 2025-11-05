/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "orbitalMotion.hpp"
#include "astroConstants.h"

constexpr double tolerance = 1e-8;

/**
 * @brief Computes the Hill frame direction cosine matrix from inertial position and velocity.
 * @param rc_N Position vector in inertial frame.
 * @param vc_N Velocity vector in inertial frame.
 * @return 3x3 DCM that transforms Hill frame vectors to inertial frame.
 */
Eigen::Matrix3d OrbitalMotion::hillFrameDCM(const Eigen::Vector3d& rc_N, const Eigen::Vector3d& vc_N) {
    const Eigen::Vector3d ir_N = rc_N.normalized();
    const Eigen::Vector3d hVec_N = rc_N.cross(vc_N);
    const Eigen::Vector3d ih_N = hVec_N.normalized();
    const Eigen::Vector3d itheta_N = ih_N.cross(ir_N);

    Eigen::Matrix3d HN;
    HN.row(0) = ir_N.transpose();
    HN.row(1) = itheta_N.transpose();
    HN.row(2) = ih_N.transpose();
    return HN;
}

/**
 * @brief Converts relative Hill frame position and velocity to inertial frame coordinates.
 * @param rc_N Chief satellite inertial position.
 * @param vc_N Chief satellite inertial velocity.
 * @param rho_H Deputy satellite relative position in Hill frame.
 * @param rhoPrime_H Deputy satellite relative velocity in Hill frame.
 * @param rd_N Output: deputy inertial position.
 * @param vd_N Output: deputy inertial velocity.
 */
void OrbitalMotion::hillToInertialState(const Eigen::Vector3d& rc_N,
                                        const Eigen::Vector3d& vc_N,
                                        const Eigen::Vector3d& rho_H,
                                        const Eigen::Vector3d& rhoPrime_H,
                                        Eigen::Vector3d& rd_N,
                                        Eigen::Vector3d& vd_N) {
    const Eigen::Matrix3d HN = hillFrameDCM(rc_N, vc_N);
    const Eigen::Matrix3d NH = HN.transpose();
    const Eigen::Vector3d hVec_N = rc_N.cross(vc_N);
    double rc = rc_N.norm();
    double fDot = hVec_N.norm() / (rc * rc);
    const Eigen::Vector3d omega_HN_H(0, 0, fDot);

    const Eigen::Vector3d rho_N = NH * rho_H;
    rd_N = rc_N + rho_N;

    const Eigen::Vector3d crossTerm = omega_HN_H.cross(rho_H);
    const Eigen::Vector3d velHill = crossTerm + rhoPrime_H;
    vd_N = vc_N + NH * velHill;
}

/**
 * @brief Converts inertial position and velocity to Hill frame relative coordinates.
 * @param rc_N Chief satellite inertial position.
 * @param vc_N Chief satellite inertial velocity.
 * @param rd_N Deputy satellite inertial position.
 * @param vd_N Deputy satellite inertial velocity.
 * @param rho_H Output: relative Hill frame position.
 * @param rhoPrime_H Output: relative Hill frame velocity.
 */
void OrbitalMotion::inertialToHillState(const Eigen::Vector3d& rc_N,
                                        const Eigen::Vector3d& vc_N,
                                        const Eigen::Vector3d& rd_N,
                                        const Eigen::Vector3d& vd_N,
                                        Eigen::Vector3d& rho_H,
                                        Eigen::Vector3d& rhoPrime_H) {
    const Eigen::Matrix3d HN = hillFrameDCM(rc_N, vc_N);
    const Eigen::Vector3d hVec_N = rc_N.cross(vc_N);
    double rc = rc_N.norm();
    double fDot = hVec_N.norm() / (rc * rc);
    const Eigen::Vector3d omega_HN_H(0, 0, fDot);

    const Eigen::Vector3d rho_N = rd_N - rc_N;
    rho_H = HN * rho_N;

    const Eigen::Vector3d rhoDot_N = vd_N - vc_N;
    const Eigen::Vector3d rhoDot_H = HN * rhoDot_N;
    rhoPrime_H = rhoDot_H - omega_HN_H.cross(rho_H);
}

/**
 * @brief Converts eccentric anomaly to true anomaly.
 * @param E Eccentric anomaly in radians.
 * @param e Orbital eccentricity.
 * @return True anomaly in radians.
 */
double OrbitalMotion::eccentricToTrueAnomaly(double E, double e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    return 2.0 * std::atan2(std::sqrt(1 + e) * std::sin(E / 2), std::sqrt(1 - e) * std::cos(E / 2));
}

/**
 * @brief Converts eccentric anomaly to mean anomaly.
 * @param E Eccentric anomaly in radians.
 * @param e Orbital eccentricity.
 * @return Mean anomaly in radians.
 */
double OrbitalMotion::eccentricToMeanAnomaly(double E, double e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    return E - e * std::sin(E);
}

/**
 * @brief Converts true anomaly to eccentric anomaly.
 * @param f True anomaly in radians.
 * @param e Orbital eccentricity.
 * @return Eccentric anomaly in radians.
 */
double OrbitalMotion::trueToEccentricAnomaly(double f, double e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    return 2.0 * std::atan2(std::sqrt(1 - e) * std::sin(f / 2), std::sqrt(1 + e) * std::cos(f / 2));
}

/**
 * @brief Convert true anomaly to mean anomaly
 * @param f True anomaly in radians
 * @param e Orbital eccentricity (0 <= e < 1).
 * @return Mean anomaly in radians.
 */
double OrbitalMotion::trueToMeanAnomaly(double f, double e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    double eccentric = trueToEccentricAnomaly(f, e);
    return eccentricToMeanAnomaly(eccentric, e);
}

/**
 * @brief Converts true anomaly to hyperbolic anomaly.
 * @param f True anomaly in radians.
 * @param e Orbital eccentricity (> 1).
 * @return Hyperbolic anomaly in radians.
 */
double OrbitalMotion::trueToHyperbolicAnomaly(double f, double e) {
    assert(e > 1.0 && "Eccentricity must be > 1 for hyperbolic orbits");
    return 2.0 * std::atanh(std::sqrt((e - 1) / (e + 1)) * std::tan(f / 2));
}

/**
 * @brief Converts hyperbolic anomaly to true anomaly.
 * @param H Hyperbolic anomaly in radians.
 * @param e Orbital eccentricity (> 1).
 * @return True anomaly in radians.
 */
double OrbitalMotion::hyperbolicToTrueAnomaly(double H, double e) {
    assert(e > 1.0 && "Eccentricity must be > 1 for hyperbolic orbits");
    return 2.0 * std::atan(std::sqrt((e + 1) / (e - 1)) * std::tanh(H / 2));
}

/**
 * @brief Converts hyperbolic anomaly to mean hyperbolic anomaly.
 * @param H Hyperbolic anomaly in radians.
 * @param e Orbital eccentricity (> 1).
 * @return Mean hyperbolic anomaly in radians.
 */
double OrbitalMotion::hyperbolicToMeanAnomaly(double H, double e) {
    assert(e > 1.0 && "Eccentricity must be > 1 for hyperbolic orbits");
    return e * std::sinh(H) - H;
}

/**
 * @brief Convert mean anomaly to eccentric anomaly
 * @param M Mean anomaly in radians.
 * @param e Orbital eccentricity (0 <= e < 1).
 * @return Eccentric anomaly in radians.
 */
double OrbitalMotion::meanToEccentricAnomaly(double M, double e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    double E = M;
    for (int i = 0; i < 200; ++i) {
        double dE = (E - e * std::sin(E) - M) / (1 - e * std::cos(E));
        E -= dE;
        if (std::abs(dE) < tolerance) break;
    }
    return E;
}

/**
 * @brief Convert mean anomaly to true anomaly
 * @param M Mean anomaly in radians.
 * @param e Orbital eccentricity (0 <= e < 1).
 * @return True anomaly in radians.
 */
double OrbitalMotion::meanToTrueAnomaly(double M, double e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    double eccentric = meanToEccentricAnomaly(M, e);
    return eccentricToTrueAnomaly(eccentric, e);
}

/**
 * @brief Mean hyperbolic anomaly to hyperbolic anomaly
 * @param N Mean hyperbolic anomaly in radians.
 * @param e Orbital eccentricity (> 1).
 * @return Hyperbolic anomaly in radians.
 */
double OrbitalMotion::meanToHyperbolicAnomaly(double N, double e) {
    assert(e > 1.0 && "Eccentricity must be > 1");
    double H = std::abs(N) > 7.0 ? 7.0 * (N > 0 ? 1 : -1) : N;
    for (int i = 0; i < 200; ++i) {
        double dH = (e * std::sinh(H) - H - N) / (e * std::cosh(H) - 1);
        H -= dH;
        if (std::abs(dH) < tolerance) break;
    }
    return H;
}

/**
 * @brief Converts orbital elements to position and velocity vectors.
 * @param mu Gravitational parameter (km^3/s^2).
 * @param elements Classical orbital elements (a, e, i, Omega, omega, f).
 * @param CartesianState Output: position and velocity vectors in km and km/s.
 */
CartesianState OrbitalMotion::elementsToCartesianState(double mu, const ClassicalElements& elements) {
    double a = elements.semiMajorAxis;
    double e = elements.eccentricity;
    double i = elements.inclination;
    double Omega = elements.rightAscensionAscendingNode;
    double omega = elements.argPeriapsis;
    double f = elements.trueAnomaly;

    double p = a * (1 - e * e);
    double r = p / (1 + e * std::cos(f));
    double h = std::sqrt(mu * p);

    double cos_O = std::cos(Omega), sin_O = std::sin(Omega);
    double cos_o = std::cos(omega), sin_o = std::sin(omega);
    double cos_i = std::cos(i), sin_i = std::sin(i);
    double cos_f = std::cos(f), sin_f = std::sin(f);

    double cos_theta = cos_o * cos_f - sin_o * sin_f;
    double sin_theta = sin_o * cos_f + cos_o * sin_f;

    Eigen::Vector3d rVec{};
    rVec(0) = r * (cos_O * cos_theta - sin_O * sin_theta * cos_i);
    rVec(1) = r * (sin_O * cos_theta + cos_O * sin_theta * cos_i);
    rVec(2) = r * (sin_theta * sin_i);

    double vx = -mu / h * (cos_O * (sin_theta + e * sin_o) + sin_O * (cos_theta + e * cos_o) * cos_i);
    double vy = -mu / h * (sin_O * (sin_theta + e * sin_o) - cos_O * (cos_theta + e * cos_o) * cos_i);
    double vz = mu / h * (cos_theta + e * cos_o) * sin_i;

    const Eigen::Vector3d vVec = Eigen::Vector3d(vx, vy, vz);

    CartesianState state{};
    state.position = rVec;
    state.velocity = vVec;

    return state;
}

/**
 * @brief Converts position and velocity vectors to classical orbital elements.
 * @param mu Gravitational parameter (km^3/s^2).
 * @param rVec Position vector in km.
 * @param vVec Velocity vector in km/s.
 * @return elements : classical orbital elements (a, e, i, Omega, omega, f).
 */
ClassicalElements OrbitalMotion::cartesianStateToElements(double mu,
                                                          const Eigen::Vector3d& rVec,
                                                          const Eigen::Vector3d& vVec) {
    double r = rVec.norm();
    double v = vVec.norm();
    const Eigen::Vector3d hVec = rVec.cross(vVec);
    double h = hVec.norm();
    const Eigen::Vector3d nVec = Eigen::Vector3d::UnitZ().cross(hVec);
    const Eigen::Vector3d eVec = ((v * v - mu / r) * rVec - (rVec.dot(vVec)) * vVec) / mu;

    ClassicalElements elements{};

    elements.radiusMagnitude = r;
    elements.alpha = 2.0 / r - v * v / mu;
    elements.semiMajorAxis = std::abs(elements.alpha) > 1e-10 ? 1.0 / elements.alpha : 0.0;
    elements.eccentricity = eVec.norm();
    elements.inclination = std::acos(hVec(2) / h);

    double Omega = std::atan2(nVec(1), nVec(0));
    elements.rightAscensionAscendingNode = Omega < 0 ? Omega + 2 * M_PI : Omega;

    double omega = std::atan2(nVec.cross(eVec).dot(hVec.normalized()), nVec.dot(eVec));
    elements.argPeriapsis = omega < 0 ? omega + 2 * M_PI : omega;

    double f = std::atan2(eVec.cross(rVec).dot(hVec.normalized()), eVec.dot(rVec));
    elements.trueAnomaly = f < 0 ? f + 2 * M_PI : f;

    elements.radiusPeriapsis = h * h / mu / (1 + elements.eccentricity);
    elements.radiusApoapsis = h * h / mu / (1 - elements.eccentricity);

    return elements;
}

/**
 * @brief Computes atmospheric density based on altitude using an empirical model.
 * @param altitudeKm Altitude above Earth's surface in kilometers.
 * @return Atmospheric density in kg/m^3.
 */
double OrbitalMotion::atmosphericDensity(double altitudeKm) {
    if (altitudeKm > 1000.0) {
        double logDensity = -7e-5 * altitudeKm - 14.464;
        return std::pow(10.0, logDensity);
    }

    double val = (altitudeKm - 526.8) / 292.8563;
    double logDensity = 0.34047 * std::pow(val, 6) - 0.5889 * std::pow(val, 5) - 0.5269 * std::pow(val, 4) +
                        1.0036 * std::pow(val, 3) + 0.60713 * std::pow(val, 2) - 2.3024 * val - 12.575;

    return std::pow(10.0, logDensity);
}

/**
 * @brief Computes acceleration due to atmospheric drag.
 * @param Cd Drag coefficient.
 * @param A Cross-sectional area in m^2.
 * @param massKg Mass of the object in kilograms.
 * @param positionKm Position vector in km (used to determine altitude).
 * @param velocityKmPerSec Velocity vector in km/s.
 * @return Drag acceleration vector in km/s^2.
 */
Eigen::Vector3d OrbitalMotion::atmosphericDragAccel(double Cd,
                                                    double A,
                                                    double massKg,
                                                    const Eigen::Vector3d& positionKm,
                                                    const Eigen::Vector3d& velocityKmPerSec) {
    const double earthRadiusKm = 6378.137;
    double r = positionKm.norm();
    double v = velocityKmPerSec.norm();
    double altitudeKm = r - earthRadiusKm;

    assert(altitudeKm > 0.0 && "Altitude must be above Earth's surface");

    double rho = atmosphericDensity(altitudeKm);
    double accelerationMag = -0.5 * rho * (Cd * A / massKg) * std::pow(v * 1000.0, 2) / 1000.0;

    return (accelerationMag / v) * velocityKmPerSec;
}

/**
 * @brief Computes acceleration due to solar radiation pressure.
 * @param areaM2 Cross-sectional area in m^2.
 * @param massKg Mass of the object in kilograms.
 * @param sunVecAU Vector from object to Sun in AU.
 * @return Acceleration due to radiation pressure in km/s^2.
 */
Eigen::Vector3d OrbitalMotion::solarRadiationPressureAccel(double areaM2,
                                                           double massKg,
                                                           const Eigen::Vector3d& sunVecAU) {
    const double flux = 1372.5398;            // W/m^2
    const double speedOfLight = 299792458.0;  // m/s
    const double Cr = 1.3;                    // Reflectivity coefficient

    double distanceAU = sunVecAU.norm();
    double scalingFactor = -Cr * areaM2 * flux / (massKg * speedOfLight * std::pow(distanceAU, 3)) / 1000.0;
    return scalingFactor * sunVecAU;
}

/**
 * @brief Computes acceleration due to zonal harmonic perturbations (J2-J6).
 * @param positionKm Position vector in km.
 * @param maxJ Maximum J-term to include (between 2 and 6).
 * @param body Celestial object enum (currently only Earth is implemented).
 * @return Perturbing acceleration vector in km/s^2.
 */
Eigen::Vector3d OrbitalMotion::zonalHarmonicPerturbation(const Eigen::Vector3d& positionKm,
                                                         int maxJ,
                                                         CelestialObject body) {
    assert((maxJ > 2 || maxJ < 6) && "maxJ must be in range [2,6]");
    double mu{};
    double req{};
    double J2{}, J3{}, J4{}, J5{}, J6{};
    if (body == CelestialObject::Sun) {
        mu = MU_SUN;
        req = REQ_SUN;
    } else if (body == CelestialObject::Mercury) {
        mu = MU_MERCURY;
        req = REQ_MERCURY;
        J2 = J2_MERCURY;
    } else if (body == CelestialObject::Venus) {
        mu = MU_VENUS;
        req = REQ_VENUS;
        J2 = J2_VENUS;
    } else if (body == CelestialObject::Moon) {
        mu = MU_MOON;
        req = REQ_MOON;
    } else if (body == CelestialObject::Mars) {
        mu = MU_MARS;
        req = REQ_MARS;
        J2 = J2_MARS;
    } else if (body == CelestialObject::Jupiter) {
        mu = MU_JUPITER;
        req = REQ_JUPITER;
        J2 = J2_JUPITER;
    } else if (body == CelestialObject::Saturn) {
        mu = MU_SATURN;
        req = REQ_SATURN;
        J2 = J2_SATURN;
    } else if (body == CelestialObject::Uranus) {
        mu = MU_URANUS;
        req = REQ_URANUS;
        J2 = J2_URANUS;
    } else if (body == CelestialObject::Neptune) {
        mu = MU_NEPTUNE;
        req = REQ_NEPTUNE;
        J2 = J2_NEPTUNE;
    } else {
        mu = MU_EARTH;
        req = REQ_EARTH;
        J2 = J2_EARTH;
        J3 = J3_EARTH;
        J4 = J4_EARTH;
        J5 = J5_EARTH;
        J6 = J6_EARTH;
    }
    double x = positionKm(0), y = positionKm(1), z = positionKm(2);
    double r = positionKm.norm();
    double zr = z / r;

    Eigen::Vector3d ajtot = Eigen::Vector3d::Zero();

    if (maxJ >= 2) {
        const Eigen::Vector3d term = {
            (1.0 - 5.0 * zr * zr) * x / r, (1.0 - 5.0 * zr * zr) * y / r, (3.0 - 5.0 * zr * zr) * z / r};
        ajtot += -1.5 * J2 * mu / std::pow(r, 2) * std::pow(req / r, 2) * term;
    }
    if (maxJ >= 3) {
        const Eigen::Vector3d term = {5.0 * (7.0 * std::pow(zr, 3) - 3.0 * zr) * x / r,
                                      5.0 * (7.0 * std::pow(zr, 3) - 3.0 * zr) * y / r,
                                      -3.0 * (10.0 * zr * zr - (35.0 / 3.0) * std::pow(zr, 4) - 1.0)};
        ajtot += 0.5 * J3 * mu / std::pow(r, 2) * std::pow(req / r, 3) * term;
    }
    if (maxJ >= 4) {
        const Eigen::Vector3d term = {(3.0 - 42.0 * zr * zr + 63.0 * std::pow(zr, 4)) * x / r,
                                      (3.0 - 42.0 * zr * zr + 63.0 * std::pow(zr, 4)) * y / r,
                                      (15.0 - 70.0 * zr * zr + 63.0 * std::pow(zr, 4)) * z / r};
        ajtot += 0.625 * J4 * mu / std::pow(r, 2) * std::pow(req / r, 4) * term;
    }
    if (maxJ >= 5) {
        const Eigen::Vector3d term = {3.0 * (35.0 * zr - 210.0 * std::pow(zr, 3) + 231.0 * std::pow(zr, 5)) * x / r,
                                      3.0 * (35.0 * zr - 210.0 * std::pow(zr, 3) + 231.0 * std::pow(zr, 5)) * y / r,
                                      -(15.0 - 315.0 * zr * zr + 945.0 * std::pow(zr, 4) - 693.0 * std::pow(zr, 6))};
        ajtot += 0.125 * J5 * mu / std::pow(r, 2) * std::pow(req / r, 5) * term;
    }
    if (maxJ >= 6) {
        const Eigen::Vector3d term = {
            (35.0 - 945.0 * zr * zr + 3465.0 * std::pow(zr, 4) - 3003.0 * std::pow(zr, 6)) * x / r,
            (35.0 - 945.0 * zr * zr + 3465.0 * std::pow(zr, 4) - 3003.0 * std::pow(zr, 6)) * y / r,
            -(3003.0 * std::pow(zr, 6) - 4851.0 * std::pow(zr, 4) + 2205.0 * zr * zr - 245.0) * z / r};
        ajtot += -1.0 / 16.0 * J6 * mu / std::pow(r, 2) * std::pow(req / r, 6) * term;
    }

    return ajtot;
}

/**
 * @brief Converts classical orbital elements to equinoctial elements.
 * @param classical Input: Classical orbital elements.
 * @param EquinoctialElements Output: Converted equinoctial elements.
 */
EquinoctialElements OrbitalMotion::mapClassicalToEquinoctialElements(const ClassicalElements& classical) {
    double e = classical.eccentricity;
    double i = classical.inclination;
    double Omega = classical.rightAscensionAscendingNode;
    double omega = classical.argPeriapsis;
    double f = classical.trueAnomaly;

    double E = trueToEccentricAnomaly(f, e);
    double M = eccentricToMeanAnomaly(E, e);

    EquinoctialElements equinoctial{};
    equinoctial.a = classical.semiMajorAxis;
    equinoctial.P1 = e * std::sin(Omega + omega);
    equinoctial.P2 = e * std::cos(Omega + omega);
    equinoctial.Q1 = std::tan(i / 2.0) * std::sin(Omega);
    equinoctial.Q2 = std::tan(i / 2.0) * std::cos(Omega);
    equinoctial.l = Omega + omega + M;
    equinoctial.L = Omega + omega + f;

    return equinoctial;
}

/**
 * @brief Converts eccentric anomaly to true anomaly.
 * @param E Eccentric anomaly in radians.
 * @param e Orbital eccentricity.
 * @return True anomaly in radians.
 */
float OrbitalMotion::eccentricToTrueAnomalyF32(float E, float e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    return 2.0 * std::atan2(std::sqrt(1 + e) * std::sin(E / 2), std::sqrt(1 - e) * std::cos(E / 2));
}

/**
 * @brief Converts eccentric anomaly to mean anomaly.
 * @param E Eccentric anomaly in radians.
 * @param e Orbital eccentricity.
 * @return Mean anomaly in radians.
 */
float OrbitalMotion::eccentricToMeanAnomalyF32(float E, float e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    return E - e * std::sin(E);
}

/**
 * @brief Converts true anomaly to eccentric anomaly.
 * @param f True anomaly in radians.
 * @param e Orbital eccentricity.
 * @return Eccentric anomaly in radians.
 */
float OrbitalMotion::trueToEccentricAnomalyF32(float f, float e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    return 2.0 * std::atan2(std::sqrt(1 - e) * std::sin(f / 2), std::sqrt(1 + e) * std::cos(f / 2));
}

/**
 * @brief Convert true anomaly to mean anomaly
 * @param f True anomaly in radians
 * @param e Orbital eccentricity (0 <= e < 1).
 * @return Mean anomaly in radians.
 */
float OrbitalMotion::trueToMeanAnomalyF32(float f, float e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    float eccentric = trueToEccentricAnomalyF32(f, e);
    return eccentricToMeanAnomalyF32(eccentric, e);
}

/**
 * @brief Converts true anomaly to hyperbolic anomaly.
 * @param f True anomaly in radians.
 * @param e Orbital eccentricity (> 1).
 * @return Hyperbolic anomaly in radians.
 */
float OrbitalMotion::trueToHyperbolicAnomalyF32(float f, float e) {
    assert(e > 1.0 && "Eccentricity must be > 1 for hyperbolic orbits");
    return 2.0 * std::atanh(std::sqrt((e - 1) / (e + 1)) * std::tan(f / 2));
}

/**
 * @brief Converts hyperbolic anomaly to true anomaly.
 * @param H Hyperbolic anomaly in radians.
 * @param e Orbital eccentricity (> 1).
 * @return True anomaly in radians.
 */
float OrbitalMotion::hyperbolicToTrueAnomalyF32(float H, float e) {
    assert(e > 1.0 && "Eccentricity must be > 1 for hyperbolic orbits");
    return 2.0 * std::atan(std::sqrt((e + 1) / (e - 1)) * std::tanh(H / 2));
}

/**
 * @brief Converts hyperbolic anomaly to mean hyperbolic anomaly.
 * @param H Hyperbolic anomaly in radians.
 * @param e Orbital eccentricity (> 1).
 * @return Mean hyperbolic anomaly in radians.
 */
float OrbitalMotion::hyperbolicToMeanAnomalyF32(float H, float e) {
    assert(e > 1.0 && "Eccentricity must be > 1 for hyperbolic orbits");
    return e * std::sinh(H) - H;
}

/**
 * @brief Convert mean anomaly to eccentric anomaly
 * @param M Mean anomaly in radians.
 * @param e Orbital eccentricity (0 <= e < 1).
 * @return Eccentric anomaly in radians.
 */
float OrbitalMotion::meanToEccentricAnomalyF32(float M, float e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    float E = M;
    for (int i = 0; i < 200; ++i) {
        float dE = (E - e * std::sin(E) - M) / (1 - e * std::cos(E));
        E -= dE;
        if (std::abs(dE) < tolerance) break;
    }
    return E;
}

/**
 * @brief Convert mean anomaly to true anomaly
 * @param M Mean anomaly in radians.
 * @param e Orbital eccentricity (0 <= e < 1).
 * @return True anomaly in radians.
 */
float OrbitalMotion::meanToTrueAnomalyF32(float M, float e) {
    assert((e >= 0.0 || e < 1.0) && "Eccentricity out of bounds (0 <= e < 1)");
    float eccentric = meanToEccentricAnomalyF32(M, e);
    return eccentricToTrueAnomalyF32(eccentric, e);
}

/**
 * @brief Mean hyperbolic anomaly to hyperbolic anomaly
 * @param N Mean hyperbolic anomaly in radians.
 * @param e Orbital eccentricity (> 1).
 * @return Hyperbolic anomaly in radians.
 */
float OrbitalMotion::meanToHyperbolicAnomalyF32(float N, float e) {
    assert(e > 1.0 && "Eccentricity must be > 1");
    float H = std::abs(N) > 7.0 ? 7.0 * (N > 0 ? 1 : -1) : N;
    for (int i = 0; i < 200; ++i) {
        float dH = (e * std::sinh(H) - H - N) / (e * std::cosh(H) - 1);
        H -= dH;
        if (std::abs(dH) < tolerance) break;
    }
    return H;
}

/**
 * @brief Converts orbital elements to position and velocity vectors.
 * @param mu Gravitational parameter (km^3/s^2).
 * @param elements Classical orbital elements (a, e, i, Omega, omega, f).
 * @param CartesianState Output: position and velocity vectors in km and km/s.
 */
CartesianState OrbitalMotion::elementsToCartesianStateF32(double mu, const ClassicalElementsF32& elements) {
    double a = elements.semiMajorAxis;
    float e = elements.eccentricity;
    float i = elements.inclination;
    float Omega = elements.rightAscensionAscendingNode;
    float omega = elements.argPeriapsis;
    float f = elements.trueAnomaly;

    double p = a * (1 - e * e);
    double r = p / (1 + e * std::cos(f));
    double h = std::sqrt(mu * p);

    float cos_O = std::cos(Omega);
    float sin_O = std::sin(Omega);
    float cos_o = std::cos(omega);
    float sin_o = std::sin(omega);
    float cos_i = std::cos(i);
    float sin_i = std::sin(i);
    float cos_f = std::cos(f);
    float sin_f = std::sin(f);

    float cos_theta = cos_o * cos_f - sin_o * sin_f;
    float sin_theta = sin_o * cos_f + cos_o * sin_f;

    Eigen::Vector3d rVec{};
    rVec(0) = r * (cos_O * cos_theta - sin_O * sin_theta * cos_i);
    rVec(1) = r * (sin_O * cos_theta + cos_O * sin_theta * cos_i);
    rVec(2) = r * (sin_theta * sin_i);

    double vx = -mu / h * (cos_O * (sin_theta + e * sin_o) + sin_O * (cos_theta + e * cos_o) * cos_i);
    double vy = -mu / h * (sin_O * (sin_theta + e * sin_o) - cos_O * (cos_theta + e * cos_o) * cos_i);
    double vz = mu / h * (cos_theta + e * cos_o) * sin_i;

    const Eigen::Vector3d vVec = Eigen::Vector3d(vx, vy, vz);

    CartesianState state{};
    state.position = rVec;
    state.velocity = vVec;

    return state;
}

/**
 * @brief Converts position and velocity vectors to classical orbital elements.
 * @param mu Gravitational parameter (km^3/s^2).
 * @param rVec Position vector in km.
 * @param vVec Velocity vector in km/s.
 * @return elements : classical orbital elements (a, e, i, Omega, omega, f).
 */
ClassicalElementsF32 OrbitalMotion::cartesianStateToElementsF32(double mu,
                                                                const Eigen::Vector3d& rVec,
                                                                const Eigen::Vector3d& vVec) {
    double r = rVec.norm();
    double v = vVec.norm();
    const Eigen::Vector3d hVec = rVec.cross(vVec);
    double h = hVec.norm();
    const Eigen::Vector3d nVec = Eigen::Vector3d::UnitZ().cross(hVec);
    const Eigen::Vector3d eVec = ((v * v - mu / r) * rVec - (rVec.dot(vVec)) * vVec) / mu;

    ClassicalElementsF32 elements{};

    elements.radiusMagnitude = r;
    elements.alpha = static_cast<float>(2.0 / r - v * v / mu);
    elements.semiMajorAxis = std::abs(elements.alpha) > 1e-10 ? 1.0 / elements.alpha : 0.0;
    elements.eccentricity = static_cast<float>(eVec.norm());
    elements.inclination = static_cast<float>(std::acos(hVec(2) / h));

    auto Omega = static_cast<float>(std::atan2(nVec(1), nVec(0)));
    elements.rightAscensionAscendingNode = Omega < 0 ? static_cast<float>(Omega + 2 * M_PI) : Omega;

    auto omega = static_cast<float>(std::atan2(nVec.cross(eVec).dot(hVec.normalized()), nVec.dot(eVec)));
    elements.argPeriapsis = omega < 0 ? static_cast<float>(omega + 2 * M_PI) : omega;

    auto f = static_cast<float>(std::atan2(eVec.cross(rVec).dot(hVec.normalized()), eVec.dot(rVec)));
    elements.trueAnomaly = f < 0 ? static_cast<float>(f + 2 * M_PI) : f;

    elements.radiusPeriapsis = h * h / mu / (1 + elements.eccentricity);
    elements.radiusApoapsis = h * h / mu / (1 - elements.eccentricity);

    return elements;
}
