// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

/*! @brief Top level structure for the flyby OD unscented kalman filter.
 Used to estimate the spacecraft's inertial position relative to a body.
 */

#ifndef FLYBYODUKF_H
#define FLYBYODUKF_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/FilterMsgPayload.h>
#include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/msgPayloadDef/OpNavUnitVecMsgPayload.h>

#include <Eigen/Core>

#include <cstdint>
#include <memory>

// Pimpl onto the framework-agnostic algorithm. Forward-declared (rather than
// included) so this header — which SWIG parses — never pulls in the
// concept-heavy filtering_core templates. The full type is included in the
// .cpp only.
namespace filtering::flybyODuKF {
class FlybyODuKFAlgorithm;
}

/*! @brief xmera host adapter for the flyby OD SRUKF. Owns the message surface
 and marshals payloads in/out of the pure-C++ FlybyODuKFAlgorithm. */
class FlybyODuKF : public SysModel {
   public:
    FlybyODuKF();
    ~FlybyODuKF();

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

   public:
    ReadFunctor<OpNavUnitVecMsgPayload> opNavHeadingMsg;
    OpNavUnitVecMsgPayload opNavHeadingBuffer;
    Message<NavTransMsgPayload> navTransOutMsg;
    Message<FilterMsgPayload> opNavFilterMsg;
    Message<FilterResidualsMsgPayload> opNavResidualMsg;

    // ---- Configuration (forwarders onto the algorithm) ----------------------

    void setAlpha(double alpha);
    double getAlpha() const;

    void setBeta(double beta);
    double getBeta() const;

    void setUnitConversionFromSItoState(double conversion);
    double getUnitConversionFromSItoState() const;

    void setCentralBodyGravitationParameter(double mu);
    double getCentralBodyGravitationParameter() const;

    void setMeasurementNoiseScale(double measurementNoiseScale);
    double getMeasurementNoiseScale() const;

    void setInitialPosition(const Eigen::Vector3d& r_BN_N);
    Eigen::Vector3d getInitialPosition() const;

    void setInitialVelocity(const Eigen::Vector3d& v_BN_N);
    Eigen::Vector3d getInitialVelocity() const;

    void setInitialCovariance(const Eigen::MatrixXd& covariance);
    Eigen::MatrixXd getInitialCovariance() const;

    void setProcessNoise(const Eigen::MatrixXd& processNoise);
    Eigen::MatrixXd getProcessNoise() const;

   private:
    void readFilterMeasurements();
    void writeOutputMessages(uint64_t currentSimNanos, bool measurementProcessed);

   private:
    std::unique_ptr<filtering::flybyODuKF::FlybyODuKFAlgorithm> algorithm;
    uint64_t previousSimNanos = 0;
};

#endif
