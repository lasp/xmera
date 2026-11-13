// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
//
// Conservation fuzz test: over a short, torque-free, fixed-small-step integrated
// run, total angular momentum and energy must not drift beyond truncation-error
// bounds, for a fuzzed reaction-wheel + hub configuration. A real conservation
// defect (sign error, momentum leak) drifts O(1) and trips these immediately.
//
// Tolerances are truncation-honest, NOT machine epsilon: an explicit RK4
// integrator conserves momentum near round-off but energy only to O(dt^5)
// accumulation. NOTE: these bounds were not executed in-repo (FuzzTest is
// FetchContent-fetched only under the fuzz presets); run `fuzz-smoke-test` to
// confirm, and if momentum 1e-12 is too tight for some fuzzed config, relax
// kMomRelTol toward the integrated test's proven 1e-10.

#include "reactionWheelTestHelpers.hpp"
#include <architecture/msgPayloadDef/RWConfigMsgPayload.h>

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>
#include <simulation/dynamics/_GeneralModuleFiles/gravityEffector.h>
#include <simulation/dynamics/reactionWheels/reactionWheelStateEffector.h>
#include <simulation/dynamics/spacecraft/spacecraft.h>

#include <cmath>
#include <cstdint>
#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace {
    using rwtest::makeRW;

    constexpr double kDt = 1e-5;             // [s] fixed integration step
    constexpr uint64_t kStepNanos = 10'000;  // 1e-5 s in ns
    constexpr int kSteps = 100;              // 1 ms total
    constexpr double kMomRelTol = 1e-12;     // angular momentum: structural, near round-off
    constexpr double kEnergyRelTol = 1e-8;   // energy: RK4 truncation-limited

    //! Owns the spacecraft and everything that must outlive it.
    struct Sim {
        std::unique_ptr<Spacecraft> sc;
        std::unique_ptr<ReactionWheelStateEffector> rw;
        std::vector<std::shared_ptr<RWConfigMsgPayload>> configs;
        std::shared_ptr<GravBodyData> earth;
    };

    void expectScalarConserved(double initial, double final, double relTol, char const* what) {
        double const denom = std::max(std::abs(initial), 1e-12);
        EXPECT_LE(std::abs(final - initial) / denom, relTol) << what << " drifted: " << initial << " -> " << final;
    }

    void
    expectVecConserved(Eigen::Vector3d const &initial, Eigen::Vector3d const &final, double relTol, char const* what) {
        double const denom = std::max(initial.norm(), 1e-9);
        EXPECT_LE((final - initial).norm() / denom, relTol) << what << " drifted";
    }

    //! n=3 reaction wheels on the body axes; `model` applies to all. Fully-coupled
    //! gets a small static imbalance so reset()'s d = U_s/mass is exercised.
    void conservationHolds(
        double hubMass,
        double i1,
        double i2,
        double i3,
        double s1,
        double s2,
        double s3,
        double w1,
        double w2,
        double w3,
        double omega0,
        double omega1,
        double omega2,
        int modelSel
    ) {
        RWModels const model = (modelSel == 0) ? BalancedWheels : (modelSel == 1) ? JitterSimple : JitterFullyCoupled;

        Sim s;
        s.sc = std::make_unique<Spacecraft>();
        s.rw = std::make_unique<ReactionWheelStateEffector>();

        // Hub: fuzzed mass/inertia/attitude/rates; orbit ICs fixed (a benign LEO state)
        // so orbital conservation stays well-conditioned and only the rotational/wheel
        // configuration is fuzzed.
        s.sc->hub.mHub = hubMass;
        Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
        I(0, 0) = i1;
        I(1, 1) = i2;
        I(2, 2) = i3;
        s.sc->hub.IHubPntBc_B = I;
        s.sc->hub.r_CN_NInit << -4020338.690396649, 7490566.741852513, 5248299.211589362;
        s.sc->hub.v_CN_NInit << -5199.77710904224, -3436.681645356935, 1041.576797498721;
        s.sc->hub.sigma_BNInit << s1, s2, s3;
        s.sc->hub.omega_BN_BInit << w1, w2, w3;

        // Central-body gravity (point mass): central force -> no torque about C or N.
        s.earth = std::make_shared<GravBodyData>();
        s.earth->planetName = "earth_planet_data";
        s.earth->mu = 0.3986004415e15;
        s.earth->isCentralBody = true;
        s.sc->gravField.gravBodies.push_back(s.earth);

        Eigen::Vector3d const axes[3] = {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()};
        double const speeds[3] = {omega0, omega1, omega2};
        for (int i = 0; i < 3; ++i) {
            auto cfg = std::make_shared<RWConfigMsgPayload>(makeRW(model, axes[i]));
            cfg->Omega = speeds[i];
            if (model == JitterFullyCoupled) {
                cfg->U_s = 1e-4;  // static imbalance -> nonzero CoM offset d
                cfg->U_d = 1e-5;
            }
            s.rw->addReactionWheel(cfg.get());
            s.configs.push_back(cfg);
        }

        s.sc->addStateEffector(s.rw.get());
        s.sc->reset(0);

        // Torque-free, friction off (defaults): the system is conservative.
        s.sc->computeEnergyMomentum(0.0);
        Eigen::Vector3d const Lorb0 = s.sc->totOrbAngMomPntN_N;
        Eigen::Vector3d const Lrot0 = s.sc->totRotAngMomPntC_N;
        double const Eorb0 = s.sc->totOrbEnergy;
        double const Erot0 = s.sc->totRotEnergy;

        uint64_t simNanos = 0;
        for (int i = 0; i < kSteps; ++i) {
            simNanos += kStepNanos;
            s.sc->updateState(simNanos);
        }

        s.sc->computeEnergyMomentum(kDt * kSteps);
        ASSERT_TRUE(s.sc->totOrbAngMomPntN_N.allFinite() && s.sc->totRotAngMomPntC_N.allFinite());
        ASSERT_TRUE(std::isfinite(s.sc->totOrbEnergy) && std::isfinite(s.sc->totRotEnergy));

        expectVecConserved(Lorb0, s.sc->totOrbAngMomPntN_N, kMomRelTol, "orbital angular momentum");
        expectVecConserved(Lrot0, s.sc->totRotAngMomPntC_N, kMomRelTol, "rotational angular momentum");
        expectScalarConserved(Eorb0, s.sc->totOrbEnergy, kEnergyRelTol, "orbital energy");
        expectScalarConserved(Erot0, s.sc->totRotEnergy, kEnergyRelTol, "rotational energy");
    }

    FUZZ_TEST(ReactionWheelConservationFuzz, conservationHolds)
        .WithDomains(
            fuzztest::InRange(50.0, 500.0),    // hub mass
            fuzztest::InRange(50.0, 1000.0),   // I1
            fuzztest::InRange(50.0, 1000.0),   // I2
            fuzztest::InRange(50.0, 1000.0),   // I3
            fuzztest::InRange(-0.3, 0.3),      // sigma_BN 1
            fuzztest::InRange(-0.3, 0.3),      // sigma_BN 2
            fuzztest::InRange(-0.3, 0.3),      // sigma_BN 3
            fuzztest::InRange(-0.1, 0.1),      // omega_BN x
            fuzztest::InRange(-0.1, 0.1),      // omega_BN y
            fuzztest::InRange(-0.1, 0.1),      // omega_BN z
            fuzztest::InRange(-500.0, 500.0),  // wheel 0 speed [rad/s]
            fuzztest::InRange(-500.0, 500.0),  // wheel 1 speed
            fuzztest::InRange(-500.0, 500.0),  // wheel 2 speed
            fuzztest::InRange(0, 2)
        );  // model selector
}  // namespace
