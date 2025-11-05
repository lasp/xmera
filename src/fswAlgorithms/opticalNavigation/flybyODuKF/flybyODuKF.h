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
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/orbitalMotion.h>

#include <fswAlgorithms/_GeneralModuleFiles/kalmanFilter.h>
#include <fswAlgorithms/_GeneralModuleFiles/measurementModels.h>
#include <fswAlgorithms/_GeneralModuleFiles/srukfInterface.h>

class FlybyODuKF : public SysModel {
   public:
    FlybyODuKF() = default;
    ~FlybyODuKF() = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

   private:
    void customReset();
    void readFilterMeasurements();
    void writeOutputMessages(uint64_t currentSimNanos);

   public:
    ReadFunctor<OpNavUnitVecMsgPayload> opNavHeadingMsg;
    OpNavUnitVecMsgPayload opNavHeadingBuffer;
    Message<NavTransMsgPayload> navTransOutMsg;
    Message<FilterMsgPayload> opNavFilterMsg;
    Message<FilterResidualsMsgPayload> opNavResidualMsg;

    void setCentralBodyGravitationParameter(double mu);
    double getCentralBodyGravitationParameter() const;

   private:
    SRukfInterface srukf{};
    std::optional<MeasurementModel> measurement = std::nullopt;
    double muCentral = 1;  //!< [GM] gravitation parameter of central body
    uint64_t previousSimNanos = 0;
};

#endif
