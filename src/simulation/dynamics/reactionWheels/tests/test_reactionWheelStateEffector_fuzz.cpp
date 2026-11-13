// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
//
// Property/invariant fuzz tests for ReactionWheelStateEffector. These assert
// module invariants (caps hold, outputs stay finite) across wide input domains —
// never golden expected values.

#include "reactionWheelTestHelpers.hpp"
#include <architecture/msgPayloadDef/RWCmdMsgPayload.h>
#include <architecture/msgPayloadDef/RWConfigMsgPayload.h>

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>
#include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
#include <simulation/dynamics/_GeneralModuleFiles/stateEffector.h>
#include <simulation/dynamics/reactionWheels/reactionWheelStateEffector.h>

#include <cmath>
#include <Eigen/Dense>

namespace {
    using rwtest::makeRW;

    Eigen::Vector3d unitFromAngles(double polar, double azimuth) {
        double const s = std::sin(polar);
        return {s * std::cos(azimuth), s * std::sin(azimuth), std::cos(polar)};
    }

    BackSubMatrices zeroBackSub() {
        BackSubMatrices b;
        b.matrixA.setZero();
        b.matrixB.setZero();
        b.matrixC.setZero();
        b.matrixD.setZero();
        b.vecTrans.setZero();
        b.vecRot.setZero();
        return b;
    }

    //! Invariant: after ConfigureRWRequests the commanded torque is finite and
    //! respects the torque and power caps, for any finite request/limits — including
    //! disabled limits (negative), zero speed, and zero power. (Locks in the guarded
    //! power divide: it only triggers when |u_cmd*Omega| >= P_max, so Omega != 0.)
    void saturationInvariants(double u_cmd, double u_max, double u_min, double pMax, double omega, double omegaMax) {
        ReactionWheelStateEffector rw;
        RWConfigMsgPayload cfg = makeRW(BalancedWheels, Eigen::Vector3d::UnitX());
        cfg.u_max = u_max;
        cfg.u_min = u_min;
        cfg.P_max = pMax;
        cfg.Omega = omega;
        cfg.Omega_max = omegaMax;
        rw.addReactionWheel(&cfg);

        rw.NewRWCmds.clear();
        RWCmdMsgPayload cmd{};
        cmd.u_cmd = u_cmd;
        rw.NewRWCmds.push_back(cmd);

        rw.ConfigureRWRequests(0.0);
        double const uc = cfg.u_current;

        ASSERT_TRUE(std::isfinite(uc));
        if (u_max > 0.0) { EXPECT_LE(std::abs(uc), u_max * (1.0 + 1e-12) + 1e-12); }
        if (pMax > 0.0) { EXPECT_LE(std::abs(uc * omega), pMax * (1.0 + 1e-9) + 1e-9); }
    }

    FUZZ_TEST(ReactionWheelFuzz, saturationInvariants)
        .WithDomains(
            fuzztest::InRange(-10.0, 10.0),    // u_cmd
            fuzztest::InRange(-1.0, 5.0),      // u_max (<=0 disables)
            fuzztest::InRange(0.0, 1.0),       // u_min
            fuzztest::InRange(-1.0, 50.0),     // P_max (<=0 disables)
            fuzztest::InRange(-700.0, 700.0),  // Omega [rad/s]
            fuzztest::InRange(-1.0, 700.0)
        );  // Omega_max (<=0 disables)

    //! Invariant: the Stribeck/Coulomb/viscous friction torque stays finite for any
    //! finite friction parameters, including the edge values the "guarded divide"
    //! analysis relies on (betaStatic <= 0, omegaLimitCycle == 0).
    void frictionStaysFinite(
        double omega,
        double omegaBefore,
        double fStatic,
        double fCoulomb,
        double betaStatic,
        double cViscous,
        double omegaLimitCycle
    ) {
        ReactionWheelStateEffector rw;
        RWConfigMsgPayload cfg = makeRW(BalancedWheels, Eigen::Vector3d::UnitX());
        cfg.Omega = omega;
        cfg.omegaBefore = omegaBefore;
        cfg.fStatic = fStatic;
        cfg.fCoulomb = fCoulomb;
        cfg.betaStatic = betaStatic;
        cfg.cViscous = cViscous;
        cfg.omegaLimitCycle = omegaLimitCycle;
        cfg.frictionStribeck = false;
        rw.addReactionWheel(&cfg);

        DynParamManager dpm;
        rwtest::linkAndRegister(dpm, rw);

        BackSubMatrices bsc = zeroBackSub();
        rw.updateContributions(0.0, bsc, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

        EXPECT_TRUE(std::isfinite(cfg.frictionTorque));
    }

    FUZZ_TEST(ReactionWheelFuzz, frictionStaysFinite)
        .WithDomains(
            fuzztest::InRange(-700.0, 700.0),  // Omega
            fuzztest::InRange(-700.0, 700.0),  // omegaBefore
            fuzztest::InRange(0.0, 1.0),       // fStatic
            fuzztest::InRange(0.0, 1.0),       // fCoulomb
            fuzztest::InRange(-1.0, 1.0),      // betaStatic (<=0 must stay finite)
            fuzztest::InRange(0.0, 1.0),       // cViscous
            fuzztest::InRange(0.0, 1.0)
        );  // omegaLimitCycle (0 must stay finite)

    //! Invariant: a fully-coupled wheel's mass properties, back-substitution
    //! contributions, and state derivatives are all finite for physically-valid
    //! configs. mass is constrained > 0 (reset() computes d = U_s/mass).
    void fullyCoupledStaysFinite(
        double js,
        double jt,
        double jg,
        double mass,
        double uS,
        double uD,
        double polar,
        double azimuth,
        double theta,
        double omega,
        double wx,
        double wy,
        double wz,
        double s1,
        double s2,
        double s3
    ) {
        ReactionWheelStateEffector rw;
        RWConfigMsgPayload cfg = makeRW(JitterFullyCoupled, unitFromAngles(polar, azimuth));
        cfg.Js = js;
        cfg.Jt = jt;
        cfg.Jg = jg;
        cfg.mass = mass;
        cfg.U_s = uS;
        cfg.U_d = uD;
        cfg.Omega = omega;
        rw.addReactionWheel(&cfg);

        DynParamManager dpm;
        rwtest::registerHubStates(dpm);
        Eigen::MatrixXd omegaBN(3, 1);
        omegaBN << wx, wy, wz;
        Eigen::MatrixXd sigmaBN(3, 1);
        sigmaBN << s1, s2, s3;
        dpm.getStateObject("hubOmega")->setState(omegaBN);
        dpm.getStateObject("hubSigma")->setState(sigmaBN);
        rw.linkInStates(dpm);
        rw.registerStates(dpm);

        // Drive the registered states to the fuzzed wheel speed / angle.
        Eigen::MatrixXd om(1, 1);
        om << omega;
        dpm.getStateObject(rw.nameOfReactionWheelOmegasState)->setState(om);
        Eigen::MatrixXd th(1, 1);
        th << theta;
        dpm.getStateObject(rw.nameOfReactionWheelThetasState)->setState(th);

        rw.reset(0);
        rw.updateEffectorMassProps(0.0);
        BackSubMatrices bsc = zeroBackSub();
        rw.updateContributions(0.0, bsc, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        rw.computeDerivatives(0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

        EXPECT_TRUE(std::isfinite(rw.effProps.mEff));
        EXPECT_TRUE(rw.effProps.rEff_CB_B.allFinite());
        EXPECT_TRUE(rw.effProps.IEffPntB_B.allFinite());
        EXPECT_TRUE(rw.effProps.rEffPrime_CB_B.allFinite());
        EXPECT_TRUE(rw.effProps.IEffPrimePntB_B.allFinite());
        EXPECT_TRUE(
            bsc.matrixA.allFinite() && bsc.matrixB.allFinite() && bsc.matrixC.allFinite() && bsc.matrixD.allFinite()
        );
        EXPECT_TRUE(bsc.vecTrans.allFinite() && bsc.vecRot.allFinite());
        EXPECT_TRUE(dpm.getStateObject(rw.nameOfReactionWheelOmegasState)->getStateDeriv().allFinite());
        EXPECT_TRUE(dpm.getStateObject(rw.nameOfReactionWheelThetasState)->getStateDeriv().allFinite());
    }

    FUZZ_TEST(ReactionWheelFuzz, fullyCoupledStaysFinite)
        .WithDomains(
            fuzztest::InRange(1e-3, 1.0),          // Js
            fuzztest::InRange(1e-3, 1.0),          // Jt
            fuzztest::InRange(1e-3, 1.0),          // Jg
            fuzztest::InRange(0.1, 50.0),          // mass (> 0)
            fuzztest::InRange(-0.1, 0.1),          // U_s
            fuzztest::InRange(-0.1, 0.1),          // U_d
            fuzztest::InRange(0.0, 3.14159),       // polar
            fuzztest::InRange(0.0, 6.28318),       // azimuth
            fuzztest::InRange(-3.14159, 3.14159),  // theta
            fuzztest::InRange(-700.0, 700.0),      // Omega
            fuzztest::InRange(-1.0, 1.0),          // hub omega x
            fuzztest::InRange(-1.0, 1.0),          // hub omega y
            fuzztest::InRange(-1.0, 1.0),          // hub omega z
            fuzztest::InRange(-1.0, 1.0),          // hub sigma 1
            fuzztest::InRange(-1.0, 1.0),          // hub sigma 2
            fuzztest::InRange(-1.0, 1.0)
        );  // hub sigma 3
}  // namespace
