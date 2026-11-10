// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef REACTION_WHEEL_TEST_HELPERS_HPP
#define REACTION_WHEEL_TEST_HELPERS_HPP

#include <architecture/msgPayloadDef/RWConfigMsgPayload.h>

#include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
#include <simulation/dynamics/reactionWheels/reactionWheelStateEffector.h>

#include <Eigen/Dense>

namespace rwtest {
    //! Build an RW config with a chosen model and spin axis, using the payload's
    //! own field defaults for everything else (mass, inertias, friction off, etc.).
    inline RWConfigMsgPayload makeRW(RWModels model, Eigen::Vector3d const &gsHat) {
        RWConfigMsgPayload rw{};
        rw.RWModel = model;
        rw.rWB_B.setZero();
        rw.gsHat_B = gsHat;
        // Pick body-fixed transverse axes orthogonal to gsHat for the jitter frame.
        Eigen::Vector3d ref = (std::abs(gsHat.z()) < 0.9) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
        rw.w2Hat0_B = gsHat.cross(ref).normalized();
        rw.w3Hat0_B = gsHat.cross(rw.w2Hat0_B).normalized();
        // Reasonable wheel inertias (Honeywell-HR16-ish) so derived dynamics are well posed.
        rw.Js = 0.159;
        rw.Jt = 0.5 * rw.Js;
        rw.Jg = 0.5 * rw.Js;
        rw.mass = 12.0;
        return rw;
    }

    //! Register the hub states the effector links to, all zeroed
    //! (state and derivative), so updateContributions/computeDerivatives can run
    //! without a full spacecraft. The caller owns `dpm`; do NOT copy it after
    //! registering (the effector stores pointers into its state map).
    inline void registerHubStates(DynParamManager &dpm) {
        Eigen::MatrixXd const zero3 = Eigen::MatrixXd::Zero(3, 1);
        for (char const* name : {"hubSigma", "hubOmega"}) {
            StateData* s = dpm.registerState(3, 1, name);
            s->setState(zero3);
            s->setDerivative(zero3);
        }
    }

    //! Add the configs, then register + link the effector against a hub-populated
    //! manager. Wheels must be added before calling (registerStates sizes the
    //! omega/theta states from the wheel list).
    inline void linkAndRegister(DynParamManager &dpm, ReactionWheelStateEffector &rw) {
        registerHubStates(dpm);
        rw.linkInStates(dpm);
        rw.registerStates(dpm);
    }
}  // namespace rwtest

#endif  // REACTION_WHEEL_TEST_HELPERS_HPP
