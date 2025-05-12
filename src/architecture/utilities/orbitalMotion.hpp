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

#ifndef ORBITAL_MOTION_HPP
#define ORBITAL_MOTION_HPP

#include <Eigen/Dense>

struct CartesianState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};

enum class CelestialObject { Sun, Mercury, Venus, Earth, Moon, Mars, Jupiter, Saturn, Uranus, Neptune, Pluto };

class ClassicalElements {
   public:
    double semiMajorAxis = 0, eccentricity = 0, inclination = 0, rightAscensionAscendingNode = 0, argPeriapsis = 0,
           trueAnomaly = 0;
    double radiusMagnitude = 0, alpha = 0, radiusPeriapsis = 0, radiusApoapsis = 0;
};

class EquinoctialElements {
   public:
    double a = 0, P1 = 0, P2 = 0, Q1 = 0, Q2 = 0, l = 0, L = 0;
};

class OrbitalMotion {
   public:
    static Eigen::Matrix3d hillFrameDCM(const Eigen::Vector3d& rc_N, const Eigen::Vector3d& vc_N);
    static void hillToInertialState(const Eigen::Vector3d& rc_N,
                                    const Eigen::Vector3d& vc_N,
                                    const Eigen::Vector3d& rho_H,
                                    const Eigen::Vector3d& rhoPrime_H,
                                    Eigen::Vector3d& rd_N,
                                    Eigen::Vector3d& vd_N);
    static void inertialToHillState(const Eigen::Vector3d& rc_N,
                                    const Eigen::Vector3d& vc_N,
                                    const Eigen::Vector3d& rd_N,
                                    const Eigen::Vector3d& vd_N,
                                    Eigen::Vector3d& rho_H,
                                    Eigen::Vector3d& rhoPrime_H);

    static double eccentricToTrueAnomaly(double E, double e);
    static double eccentricToMeanAnomaly(double E, double e);
    static double trueToEccentricAnomaly(double f, double e);
    static double trueToHyperbolicAnomaly(double f, double e);
    static double trueToMeanAnomaly(double f, double e);
    static double hyperbolicToTrueAnomaly(double H, double e);
    static double hyperbolicToMeanAnomaly(double H, double e);
    static double meanToEccentricAnomaly(double M, double e);
    static double meanToTrueAnomaly(double M, double e);
    static double meanToHyperbolicAnomaly(double N, double e);

    static CartesianState elementsToCartesianState(double mu, const ClassicalElements& elements);
    static ClassicalElements cartesianStateToElements(double mu,
                                                      const Eigen::Vector3d& rVec,
                                                      const Eigen::Vector3d& vVec);

    static double atmosphericDensity(double altitudeKm);
    static Eigen::Vector3d atmosphericDragAccel(double Cd,
                                                double A,
                                                double massKg,
                                                const Eigen::Vector3d& positionKm,
                                                const Eigen::Vector3d& velocityKmPerSec);

    static Eigen::Vector3d solarRadiationPressureAccel(double areaM2, double massKg, const Eigen::Vector3d& sunVecAU);
    static Eigen::Vector3d zonalHarmonicPerturbation(const Eigen::Vector3d& positionKm, int maxJ, CelestialObject body);

    static EquinoctialElements mapClassicalToEquinoctialElements(const ClassicalElements& classicals);
};

#endif
