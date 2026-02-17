// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef TIME_CA_H
#define TIME_CA_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/FilterMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/msgPayloadDef/TimeClosestApproachMsgPayload.h>
#include <architecture/utilities/eigenSupport.h>

#include <Eigen/Core>

/*! @brief A class to perform time of closest approach estimation during a rectilinear flyby */
class TimeClosestApproach : public SysModel {
   public:
    TimeClosestApproach();
    ~TimeClosestApproach() override;
    void updateState(uint64_t currentSimNanos) override;

    ReadFunctor<FilterMsgPayload> filterInMsg;  //!< relative state and covariance input msg
    ReadFunctor<NavTransMsgPayload> navFilterMsg;
    Message<TimeClosestApproachMsgPayload> tcaOutMsg;  //!< time of closest approach output message

   private:
    void readMessages();
    void computeGeometry();
    double computeTca() const;
    double computeTcaStandardDeviation() const;
    void writeMessages(double tCA, double sigmaTca, uint64_t currentSimNanos);

    Eigen::Vector3d v_BN_N;              //!< spacecraft velocity estimate in inertial coordinates
    Eigen::Vector3d r_BN_N;              //!< spacecraft position estimate in inertial coordinates
    Eigen::MatrixXd filterCovariance;    //!< filter covariance
    double flightPathAngle = -M_PI / 2;  //!< flight path angle of the spacecraft at time of read [rad]
    double ratio = 0;                    //!< ratio between relative velocity and position norms at time of read [Hz]
    int numberOfStates = 0;              //!< Number of states in the filter estimate
};

#endif
