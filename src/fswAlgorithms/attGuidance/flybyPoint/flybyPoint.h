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

#ifndef FLYBY_POINT_H
#define FLYBY_POINT_H

#include <xmera/sys_model.h>
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/AttRefMsgPayload.h"
#include "architecture/msgPayloadDef/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDef/NavTransMsgPayload.h"
#include "fswAlgorithms/attGuidance/flybyPoint/flybyPointAlgorithm.h"
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

    ReadFunctor<NavTransMsgPayload> filterInMsg;              //!< input msg relative position w.r.t. asteroid
    ReadFunctor<EphemerisMsgPayload> asteroidEphemerisInMsg;  //!< input asteroid ephemeris msg
    Message<AttRefMsgPayload> attRefOutMsg;                   //!< Attitude reference output message

   private:
    FlybyPointAlgorithm algorithm;
};

#endif
