// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FLYBY_POINT_H
#define FLYBY_POINT_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/FlybyDiagnosticMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include "flybyPointAlgorithm.h"
#include <Eigen/Dense>

/*! @brief A class to perform flyby pointing */
class FlybyPoint : public SysModel {
   public:
    FlybyPoint();
    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> readRelativeState();
    double getTimeBetweenFilterData() const;
    void setTimeBetweenFilterData(double timeBetweenFilterData);
    double getToleranceForCollinearity() const;
    void setToleranceForCollinearity(double toleranceForCollinearity);
    int getSignOfOrbitNormalFrameVector() const;
    void setSignOfOrbitNormalFrameVector(int signOfOrbitNormalFrameVector);
    double getMaximumAccelerationThreshold() const;
    void setMaximumAccelerationThreshold(double maxAccelerationThreshold);
    double getMaximumRateThreshold() const;
    void setMaximumRateThreshold(double maxRateThreshold);
    double getPositionKnowledgeSigma() const;
    void setPositionKnowledgeSigma(double positionKnowledgeStd);

    ReadFunctor<NavTransMsgPayload> filterInMsg;               //!< input msg relative position w.r.t. asteroid
    ReadFunctor<EphemerisMsgPayload> asteroidEphemerisInMsg;   //!< input asteroid ephemeris msg
    Message<AttRefMsgPayload> attRefOutMsg;                    //!< Attitude reference output message
    Message<FlybyDiagnosticMsgPayload> flybyDiagnosticOutMsg;  //!< Flyby diagnostic output message

   private:
    FlybyPointAlgorithm algorithm;
};

#endif
