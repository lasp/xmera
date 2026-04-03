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
#include <architecture/msgPayloadDef/IMUSensorMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/STAttMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include <fswAlgorithms/_GeneralModuleFiles/srukfInterface.h>

#define MAX_ST_VEH_COUNT 4

/*! @brief Star Tracker (ST) sensor container class. Contains the msg input name and Id and sensor noise value. */
class StarTrackerMessage {
   public:
    ReadFunctor<STAttMsgPayload> starTrackerMsg;  //!< star tracker input message
    Eigen::Matrix3d measurementNoise_C;           //!< [-] Per axis noise on the ST
};

enum class AttitudeFilterMethod { StarOnly, GyroWhenDazzled, AllMeasurements };

/*! @brief Inertial Attitude filter which reads Star Tracker measurements and gyro measurements */
class InertialAttitudeUkf : public SRukfInterface {
   public:
    InertialAttitudeUkf(AttitudeFilterMethod method);
    ~InertialAttitudeUkf() = default;

   private:
    void customreset() final;
    void readFilterMeasurements(uint64_t currentSimNanos) final;
    void writeOutputMessages(uint64_t currentSimNanos) final;
    void customInitializeUpdate() final;
    void customFinalizeUpdate() final;

    /*! Specific read messages */
    void readRWSpeedData();
    void readStarTrackerData();
    void readGyroData(uint64_t currentSimNanos);
    void switchStateCovariance();

   public:
    ReadFunctor<RWArrayConfigMsgPayload> rwArrayConfigMsg;
    RWArrayConfigMsgPayload rwArrayConfigPayload;
    ReadFunctor<VehicleConfigMsgPayload> vehicleConfigMsg;
    ReadFunctor<RWSpeedMsgPayload> rwSpeedMsg;
    ReadFunctor<IMUSensorMsgPayload> imuSensorDataInMsg;

    Message<NavAttMsgPayload> navAttitudeOutputMsg;
    Message<FilterMsgPayload> inertialFilterOutputMsg;
    Message<FilterResidualsMsgPayload> starTrackerResidualMsg;
    Message<FilterResidualsMsgPayload> gyroResidualMsg;

    void setGyroNoise(const Eigen::Matrix3d& gyroNoise);
    Eigen::Matrix3d getGyroNoise() const;
    void addStarTrackerInput(const StarTrackerMessage& starTracker);
    Eigen::Matrix3d getStarTrackerNoise(int starTrackerNumber) const;

   private:
    AttitudeFilterMethod measurementAcceptanceMethod;
    bool firstFilterPass = true;
    bool validStarTracker = false;

    Eigen::VectorXd wheelAccelerations{};
    Eigen::VectorXd previousWheelSpeeds{};
    Eigen::Matrix3d spacecraftInertia;
    Eigen::Matrix3d spacecraftInertiaInverse;
    double previousWheelSpeedTime = 0;

    Eigen::Vector3d filteredOmega_BN_B;
    Eigen::Matrix3d gyroNoise;
    std::array<StarTrackerMessage, MAX_ST_VEH_COUNT> starTrackerMessages;
    int numberOfStarTackers = 0;
    int measurementIndex = 0;
    double mrpSwitchThreshold = 1;  //!< [-] Threshold for switching MRP to/from the shadow set
};

#endif
