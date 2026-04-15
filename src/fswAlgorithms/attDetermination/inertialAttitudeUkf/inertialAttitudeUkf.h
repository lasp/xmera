// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

/*! @brief Top level structure for the inertial attitude unscented kalman filter.
 Used to estimate the spacecraft's inertial attitude as MRPs and attitude rate.
 */

#ifndef INERTIAL_ATTITUDE_UKF_H
#define INERTIAL_ATTITUDE_UKF_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/FilterMsgPayload.h>
#include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/STAttMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include <fswAlgorithms/_GeneralModuleFiles/srukfInterface.h>

#define MAX_ST_VEH_COUNT 4

/*! @brief Star Tracker (ST) sensor container class. Contains the msg input name and Id and sensor noise value. */
class AttitudeMessage {
   public:
    ReadFunctor<STAttMsgPayload> attitudeMsg;  //!< attitude input message
    Eigen::Matrix3d measurementNoise_C;        //!< [-] Per axis noise on the attitude
};

enum class AttitudeFilterMethod { AttitudeOnly, RateMeasurementsWhenNoStars, AllMeasurements };

/*! @brief Inertial Attitude filter which reads Star Tracker measurements and gyro measurements */
class InertialAttitudeUkf : public SRukfInterface {
   public:
    InertialAttitudeUkf(AttitudeFilterMethod method);
    ~InertialAttitudeUkf() = default;

   private:
    void customreset() final;
    void readFilterMeasurements() final;
    void writeOutputMessages(uint64_t currentSimNanos) final;
    void customInitializeUpdate() final;
    void customFinalizeUpdate() final;

    /*! Specific read messages */
    void readRWSpeedData();
    void readAttitudeData();
    void readRateData();
    void switchStateCovariance();

   public:
    ReadFunctor<RWArrayConfigMsgPayload> rwArrayConfigMsg;
    RWArrayConfigMsgPayload rwArrayConfigPayload;
    ReadFunctor<VehicleConfigMsgPayload> vehicleConfigMsg;
    ReadFunctor<RWSpeedMsgPayload> rwSpeedMsg;
    ReadFunctor<STAttMsgPayload> rateDataInMsg;

    Message<NavAttMsgPayload> navAttitudeOutputMsg;
    Message<FilterMsgPayload> inertialFilterOutputMsg;
    Message<FilterResidualsMsgPayload> attitudeResidualMsg;
    Message<FilterResidualsMsgPayload> rateResidualMsg;

    void setRateNoise(const Eigen::Matrix3d& rateNoise);
    Eigen::Matrix3d getRateNoise() const;
    void addAttitudeInput(const AttitudeMessage& attitudeMeasurement);
    Eigen::Matrix3d getAttitudeNoise(int attitudeMeasurementNumber) const;

   private:
    AttitudeFilterMethod measurementAcceptanceMethod;
    bool firstFilterPass = true;
    bool validAttitude = false;

    Eigen::VectorXd wheelAccelerations{};
    Eigen::VectorXd previousWheelSpeeds{};
    Eigen::Matrix3d spacecraftInertia;
    Eigen::Matrix3d spacecraftInertiaInverse;
    double previousWheelSpeedTime = 0;

    Eigen::Vector3d filteredOmega_BN_B;
    Eigen::Matrix3d rateNoise;
    std::array<AttitudeMessage, MAX_ST_VEH_COUNT> attitudeMessages;
    int numberOfStarTackers = 0;
    int measurementIndex = 0;
    double mrpSwitchThreshold = 1;  //!< [-] Threshold for switching MRP to/from the shadow set
};

#endif
