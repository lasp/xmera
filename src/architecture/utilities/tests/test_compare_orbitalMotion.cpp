// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "architecture/utilities/orbitalMotion.hpp"
#include <gtest/gtest.h>
extern "C" {
#include "architecture/utilities/orbitalMotion.h"
}

constexpr double tol = 1e-8;

class OrbitalMotionTest : public ::testing::Test {
   protected:
    void SetUp() override {
        // Test setup if needed
    }
};

TEST_F(OrbitalMotionTest, CompareAtmosphericDensity) {
    for (int alt = 100; alt <= 1000; alt += 100) {
        double cResult = atmosphericDensity(alt);
        double cppResult = OrbitalMotion::atmosphericDensity(alt);
        EXPECT_NEAR(cResult, cppResult, tol);
    }
}

TEST_F(OrbitalMotionTest, CompareEccentricToTrueAnomaly) {
    double start = 0.01;
    double end = 1;
    double step = 0.1;
    auto steps = static_cast<int>((end - start) / step);

    for (int i = 0; i <= steps; ++i) {
        double e = start + i * step;
        double E = 1.0;
        double cResult = E2f(E, e);
        double cppResult = OrbitalMotion::eccentricToTrueAnomaly(E, e);
        EXPECT_NEAR(cResult, cppResult, tol);
    }
}

TEST_F(OrbitalMotionTest, CompareEccentricToMeanAnomaly) {
    double start = 0.01;
    double end = 1;
    double step = 0.1;
    auto steps = static_cast<int>((end - start) / step);

    for (int i = 0; i <= steps; ++i) {
        double e = start + i * step;
        double E = 1.0;
        double cResult = E2M(E, e);
        double cppResult = OrbitalMotion::eccentricToMeanAnomaly(E, e);
        EXPECT_NEAR(cResult, cppResult, tol);
    }
}

TEST_F(OrbitalMotionTest, CompareTrueToEccentricAnomaly) {
    double start = 0.01;
    double end = 1;
    double step = 0.1;
    auto steps = static_cast<int>((end - start) / step);

    for (int i = 0; i <= steps; ++i) {
        double e = start + i * step;
        double f = 1.0;
        double cResult = f2E(f, e);
        double cppResult = OrbitalMotion::trueToEccentricAnomaly(f, e);
        EXPECT_NEAR(cResult, cppResult, tol);
    }
}

TEST_F(OrbitalMotionTest, CompareTrueToHyperbolicAnomaly) {
    double start = 1.1;
    double end = 2;
    double step = 0.2;
    auto steps = static_cast<int>((end - start) / step);

    for (int i = 0; i <= steps; ++i) {
        double e = start + i * step;
        double f = 1.0;
        double cResult = f2H(f, e);
        double cppResult = OrbitalMotion::trueToHyperbolicAnomaly(f, e);
        EXPECT_NEAR(cResult, cppResult, tol);
    }
}

TEST_F(OrbitalMotionTest, CompareHyperbolicToTrueAnomaly) {
    double start = 1.1;
    double end = 2;
    double step = 0.2;
    auto steps = static_cast<int>((end - start) / step);

    for (int i = 0; i <= steps; ++i) {
        double e = start + i * step;
        double H = 1.0;
        double cResult = H2f(H, e);
        double cppResult = OrbitalMotion::hyperbolicToTrueAnomaly(H, e);
        EXPECT_NEAR(cResult, cppResult, tol);
    }
}

TEST_F(OrbitalMotionTest, CompareHyperbolicToMeanAnomaly) {
    double start = 1.1;
    double end = 2;
    double step = 0.2;
    auto steps = static_cast<int>((end - start) / step);

    for (int i = 0; i <= steps; ++i) {
        double e = start + i * step;
        double H = 1.0;
        double cResult = H2N(H, e);
        double cppResult = OrbitalMotion::hyperbolicToMeanAnomaly(H, e);
        EXPECT_NEAR(cResult, cppResult, tol);
    }
}

TEST_F(OrbitalMotionTest, CompareSolveKeplerEquationElliptic) {
    double start = 0.01;
    double end = 1;
    double step = 0.1;
    auto steps = static_cast<int>((end - start) / step);

    for (int i = 0; i <= steps; ++i) {
        double e = start + i * step;
        double M = 1.0;
        double cResult = M2E(M, e);
        double cppResult = OrbitalMotion::meanToEccentricAnomaly(M, e);
        EXPECT_NEAR(cResult, cppResult, tol);
    }
}

TEST_F(OrbitalMotionTest, CompareSolveKeplerEquationHyperbolic) {
    double start = 1.1;
    double end = 2;
    double step = 0.2;
    auto steps = static_cast<int>((end - start) / step);

    for (int i = 0; i <= steps; ++i) {
        double e = start + i * step;
        double N = 1.0;
        double cResult = N2H(N, e);
        double cppResult = OrbitalMotion::meanToHyperbolicAnomaly(N, e);
        EXPECT_NEAR(cResult, cppResult, tol);
    }
}

TEST_F(OrbitalMotionTest, CompareHillToInertialState) {
    double rcInertial[3] = {7000, 0, 0};
    double vcInertial[3] = {0, 7.5, 0};
    double rhoHill[3] = {1, 0, 0};
    double rhoPrimeHill[3] = {0, 0.1, 0};
    double rdInertialC[3];
    double vdInertialC[3];
    hill2rv(rcInertial, vcInertial, rhoHill, rhoPrimeHill, rdInertialC, vdInertialC);

    const Eigen::Vector3d rc(rcInertial[0], rcInertial[1], rcInertial[2]);
    const Eigen::Vector3d vc(vcInertial[0], vcInertial[1], vcInertial[2]);
    const Eigen::Vector3d rho(rhoHill[0], rhoHill[1], rhoHill[2]);
    const Eigen::Vector3d rhoPrime(rhoPrimeHill[0], rhoPrimeHill[1], rhoPrimeHill[2]);
    Eigen::Vector3d rdCpp;
    Eigen::Vector3d vdCpp;

    OrbitalMotion::hillToInertialState(rc, vc, rho, rhoPrime, rdCpp, vdCpp);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(rdInertialC[i], rdCpp(i), tol);
        EXPECT_NEAR(vdInertialC[i], vdCpp(i), tol);
    }
}

TEST_F(OrbitalMotionTest, CompareInertialToHillState) {
    double rcInertial[3] = {7000, 0, 0};
    double vcInertial[3] = {0, 7.5, 0};
    double rdInertial[3] = {7001, 0, 0};
    double vdInertial[3] = {0, 7.6, 0};
    double rhoHillC[3];
    double rhoPrimeHillC[3];
    rv2hill(rcInertial, vcInertial, rdInertial, vdInertial, rhoHillC, rhoPrimeHillC);

    const Eigen::Vector3d rc(rcInertial[0], rcInertial[1], rcInertial[2]);
    const Eigen::Vector3d vc(vcInertial[0], vcInertial[1], vcInertial[2]);
    const Eigen::Vector3d rd(rdInertial[0], rdInertial[1], rdInertial[2]);
    const Eigen::Vector3d vd(vdInertial[0], vdInertial[1], vdInertial[2]);
    Eigen::Vector3d rhoHillCpp;
    Eigen::Vector3d rhoPrimeHillCpp;

    OrbitalMotion::inertialToHillState(rc, vc, rd, vd, rhoHillCpp, rhoPrimeHillCpp);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(rhoHillC[i], rhoHillCpp(i), tol);
        EXPECT_NEAR(rhoPrimeHillC[i], rhoPrimeHillCpp(i), tol);
    }
}

TEST_F(OrbitalMotionTest, CompareAtmosphericDragAccel) {
    double Cd = 2.2;
    double A = 3.0;
    double m = 1000.0;
    double rvec[3] = {7000.0, 0, 0};
    double vvec[3] = {0, 7.5, 0};
    double advec[3];
    atmosphericDrag(Cd, A, m, rvec, vvec, advec);

    const Eigen::Vector3d r(rvec[0], rvec[1], rvec[2]);
    const Eigen::Vector3d v(vvec[0], vvec[1], vvec[2]);
    const Eigen::Vector3d accCpp = OrbitalMotion::atmosphericDragAccel(Cd, A, m, r, v);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(advec[i], accCpp(i), tol);
    }
}

TEST_F(OrbitalMotionTest, CompareSolarRadiationPressureAccel) {
    double A = 3.0;
    double m = 1000.0;
    double sunvec[3] = {1.0, 0.0, 0.0};  // in AU
    double arvec[3];
    solarRad(A, m, sunvec, arvec);

    const Eigen::Vector3d sun(sunvec[0], sunvec[1], sunvec[2]);
    const Eigen::Vector3d accCpp = OrbitalMotion::solarRadiationPressureAccel(A, m, sun);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(arvec[i], accCpp(i), tol);
    }
}

TEST_F(OrbitalMotionTest, CompareElementsToCartesianState) {
    ClassicalElements cppElements;
    cppElements.semiMajorAxis = 7000;
    cppElements.eccentricity = 0.01;
    cppElements.inclination = 0.1;
    cppElements.rightAscensionAscendingNode = 0.5;
    cppElements.argPeriapsis = 0.4;
    cppElements.trueAnomaly = 1.0;

    ClassicElements cElements;
    cElements.a = cppElements.semiMajorAxis;
    cElements.e = cppElements.eccentricity;
    cElements.i = cppElements.inclination;
    cElements.Omega = cppElements.rightAscensionAscendingNode;
    cElements.omega = cppElements.argPeriapsis;
    cElements.f = cppElements.trueAnomaly;

    auto stateCpp = OrbitalMotion::elementsToCartesianState(398600.4418, cppElements);

    double rC[3];
    double vC[3];
    elem2rv(398600.4418, &cElements, rC, vC);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(rC[i], stateCpp.position(i), tol);
        EXPECT_NEAR(vC[i], stateCpp.velocity(i), tol);
    }
}

TEST_F(OrbitalMotionTest, CompareCartesianStateToElements) {
    const Eigen::Vector3d rCpp(6524.834, 6862.875, 6448.296);
    const Eigen::Vector3d vCpp(4.901327, 5.533756, -1.976341);

    double rC[3] = {rCpp(0), rCpp(1), rCpp(2)};
    double vC[3] = {vCpp(0), vCpp(1), vCpp(2)};

    ClassicElements cElements;
    ClassicalElements cppElements = OrbitalMotion::cartesianStateToElements(398600.4418, rCpp, vCpp);
    rv2elem(398600.4418, rC, vC, &cElements);

    EXPECT_NEAR(cppElements.semiMajorAxis, cElements.a, tol);
    EXPECT_NEAR(cppElements.eccentricity, cElements.e, tol);
    EXPECT_NEAR(cppElements.inclination, cElements.i, tol);
    EXPECT_NEAR(cppElements.rightAscensionAscendingNode, cElements.Omega, tol);
    EXPECT_NEAR(cppElements.argPeriapsis, cElements.omega, tol);
    EXPECT_NEAR(cppElements.trueAnomaly, cElements.f, tol);
    EXPECT_NEAR(cppElements.radiusPeriapsis, cElements.rPeriap, tol);
    EXPECT_NEAR(cppElements.radiusApoapsis, cElements.rApoap, tol);
}

TEST_F(OrbitalMotionTest, Compare_mapClassicalToEquinoctialElements) {
    ClassicalElements cppElements{};
    cppElements.semiMajorAxis = 7000;
    cppElements.eccentricity = 0.05;
    cppElements.inclination = 0.1;
    cppElements.rightAscensionAscendingNode = 1.0;
    cppElements.argPeriapsis = 0.5;
    cppElements.trueAnomaly = 0.2;

    ClassicElements cElements{};
    cElements.a = cppElements.semiMajorAxis;
    cElements.e = cppElements.eccentricity;
    cElements.i = cppElements.inclination;
    cElements.Omega = cppElements.rightAscensionAscendingNode;
    cElements.omega = cppElements.argPeriapsis;
    cElements.f = cppElements.trueAnomaly;

    equinoctialElements eqC;
    clElem2eqElem(&cElements, &eqC);

    EquinoctialElements eqCpp = OrbitalMotion::mapClassicalToEquinoctialElements(cppElements);

    EXPECT_NEAR(eqC.a, eqCpp.a, tol);
    EXPECT_NEAR(eqC.P1, eqCpp.P1, tol);
    EXPECT_NEAR(eqC.P2, eqCpp.P2, tol);
    EXPECT_NEAR(eqC.Q1, eqCpp.Q1, tol);
    EXPECT_NEAR(eqC.Q2, eqCpp.Q2, tol);
    EXPECT_NEAR(eqC.l, eqCpp.l, tol);
    EXPECT_NEAR(eqC.L, eqCpp.L, tol);
}
