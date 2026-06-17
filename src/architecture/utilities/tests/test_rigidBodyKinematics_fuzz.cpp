// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include <architecture/utilities/rigidBodyKinematics.hpp>

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>

namespace {
    //! Build a unit quaternion from four arbitrary doubles, falling back to identity near zero.
    Eigen::Vector4d unitQuaternion(double q0, double q1, double q2, double q3) {
        Eigen::Vector4d ep(q0, q1, q2, q3);
        double const n = ep.norm();
        if (n < 1e-9) { return {1.0, 0.0, 0.0, 0.0}; }
        return ep / n;
    }

    bool closeRel(Eigen::Vector3d const &a, Eigen::Vector3d const &b, double tol) {
        return (a - b).norm() <= tol * (1.0 + b.norm());
    }

    // prv -> ep -> prv round-trips for ||prv|| < pi.
    void roundTripPrvEp(double px, double py, double pz) {
        Eigen::Vector3d const prv(px, py, pz);
        Eigen::Vector3d const out = epToPrv<double>(prvToEp<double>(prv));
        ASSERT_TRUE(out.allFinite());
        EXPECT_TRUE(closeRel(out, prv, 1e-9));
    }

    FUZZ_TEST(RigidBodyKinematicsFuzz, roundTripPrvEp)
        .WithDomains(fuzztest::InRange(-1.0, 1.0), fuzztest::InRange(-1.0, 1.0), fuzztest::InRange(-1.0, 1.0));

    // mrp -> ep -> mrp round-trips for the canonical set (||mrp|| < 1).
    void roundTripMrpEp(double mx, double my, double mz) {
        Eigen::Vector3d const mrp(mx, my, mz);
        Eigen::Vector3d const out = epToMrp<double>(mrpToEp<double>(mrp));
        ASSERT_TRUE(out.allFinite());
        EXPECT_TRUE(closeRel(out, mrp, 1e-9));
    }

    FUZZ_TEST(RigidBodyKinematicsFuzz, roundTripMrpEp)
        .WithDomains(fuzztest::InRange(-0.5, 0.5), fuzztest::InRange(-0.5, 0.5), fuzztest::InRange(-0.5, 0.5));

    // Any unit EP maps to a proper orthonormal DCM.
    void epToDcmProperOrthonormal(double q0, double q1, double q2, double q3) {
        Eigen::Matrix3d const r = epToDcm<double>(unitQuaternion(q0, q1, q2, q3));
        ASSERT_TRUE(r.allFinite());
        EXPECT_LT((r * r.transpose() - Eigen::Matrix3d::Identity()).norm(), 1e-12);
        EXPECT_NEAR(r.determinant(), 1.0, 1e-12);
    }

    FUZZ_TEST(RigidBodyKinematicsFuzz, epToDcmProperOrthonormal)
        .WithDomains(
            fuzztest::InRange(-1.0, 1.0),
            fuzztest::InRange(-1.0, 1.0),
            fuzztest::InRange(-1.0, 1.0),
            fuzztest::InRange(-1.0, 1.0)
        );

    // subPrv undoes addPrv: subPrv(addPrv(b, a), b) == a (small domains keep the composition < pi).
    // PRV composition is non-commutative: addPrv(b, a) = qb (x) qa and subPrv(., b) removes qb from
    // the left, so b must be applied first for the round-trip to recover a (not a conjugation of it).
    void addSubPrvInverse(double ax, double ay, double az, double bx, double by, double bz) {
        Eigen::Vector3d const a(ax, ay, az);
        Eigen::Vector3d const b(bx, by, bz);
        Eigen::Vector3d const out = subPrv<double>(addPrv<double>(b, a), b);
        ASSERT_TRUE(out.allFinite());
        EXPECT_TRUE(closeRel(out, a, 1e-8));
    }

    FUZZ_TEST(RigidBodyKinematicsFuzz, addSubPrvInverse)
        .WithDomains(
            fuzztest::InRange(-0.5, 0.5),
            fuzztest::InRange(-0.5, 0.5),
            fuzztest::InRange(-0.5, 0.5),
            fuzztest::InRange(-0.5, 0.5),
            fuzztest::InRange(-0.5, 0.5),
            fuzztest::InRange(-0.5, 0.5)
        );

    // Euler-321 extraction is always finite, including near gimbal lock.
    void euler321ExtractionFinite(double q0, double q1, double q2, double q3) {
        Eigen::Vector4d const ep = unitQuaternion(q0, q1, q2, q3);
        EXPECT_TRUE(dcmToEulerAngles321<double>(epToDcm<double>(ep)).allFinite());
        EXPECT_TRUE(epToEulerAngles321<double>(ep).allFinite());
    }

    FUZZ_TEST(RigidBodyKinematicsFuzz, euler321ExtractionFinite)
        .WithDomains(
            fuzztest::InRange(-1.0, 1.0),
            fuzztest::InRange(-1.0, 1.0),
            fuzztest::InRange(-1.0, 1.0),
            fuzztest::InRange(-1.0, 1.0)
        );
}  // namespace
