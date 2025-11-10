// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

/*! @brief Top level structure for the inertial attitude unscented kalman filter.
 Used to estimate the spacecraft's inertial attitude as MRPs and attitude rate.
 */

#ifndef INERTIAL_ATTITUDE_UKF_H
#define INERTIAL_ATTITUDE_UKF_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>
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

enum class InertialAttitudeUkfMeasurementType { Attitude, Rate };

struct InertialAttitudeUkfMeasurementModel final : public SRukfMeasurementModel {
public:
    Eigen::MatrixXd model(const FilterStateVector& state) const override {
        switch (this->type) {
        case InertialAttitudeUkfMeasurementType::Attitude:
            return state.getPositionStates();

        case InertialAttitudeUkfMeasurementType::Rate: {
            Eigen::VectorXd observation = state.getVelocityStates();
            if (state.hasBias()) {
                assert(observation.size() == state.getBiasStates().size());
                observation += state.getBiasStates();
            }
            return observation;
        }
        }
    }

    Eigen::VectorXd subtract(
        const Eigen::VectorXd& observed,
        const Eigen::VectorXd& predicted
    ) const override {
        switch (this->type) {
        case InertialAttitudeUkfMeasurementType::Attitude: {
            Eigen::Vector3d yMeas = observed - predicted;
            if (observed.norm() > 0.95 && predicted.norm() > 0.95) {
                const Eigen::Vector3d predictedShadow = mrpShadow(Eigen::Vector3d{predicted});
                Eigen::Vector3d yMeasShadow = observed - predictedShadow;
                if (yMeasShadow.norm() < yMeas.norm()) {
                    return yMeasShadow;
                }
            }
            return yMeas;
        } break;

        case InertialAttitudeUkfMeasurementType::Rate:
            return observed - predicted;
        }
    }

    Eigen::VectorXd getObservation() const override {
        return this->observation;
    }

    Eigen::MatrixXd getNoise() const override {
        return this->measNoise;
    }

    void setPreFitResiduals(Eigen::VectorXd const& preFitResiduals) override {
        this->preFitResiduals = preFitResiduals;
    }

    void setPostFitResiduals(Eigen::VectorXd const& postFitResiduals) override {
        this->postFitResiduals = postFitResiduals;
    }

public:
    InertialAttitudeUkfMeasurementType type = InertialAttitudeUkfMeasurementType::Attitude;

    Eigen::VectorXd observation = {};       //!< [-] Observation data vector
    Eigen::MatrixXd measNoise = {};         //!< [-] Measurement noise

public:
    Eigen::VectorXd preFitResiduals = {};   //!< [-] Observation pre fit residuals
    Eigen::VectorXd postFitResiduals = {};  //!< [-] Observation post fit residuals
};

/*! @brief Inertial Attitude filter which reads Star Tracker measurements and gyro measurements */
class InertialAttitudeUkf : public SysModel {
   public:
    explicit InertialAttitudeUkf(AttitudeFilterMethod method);
    ~InertialAttitudeUkf() override = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    void setRateNoise(const Eigen::Matrix3d& rateNoise);
    Eigen::Matrix3d getRateNoise() const;
    void setMeasurementNoiseScale(double measurementNoiseScale);
    double getMeasurementNoiseScale() const;
    void addAttitudeInput(const AttitudeMessage& attitudeMeasurement);
    Eigen::Matrix3d getAttitudeNoise(int attitudeMeasurementNumber) const;

    ReadFunctor<RWArrayConfigMsgPayload> rwArrayConfigMsg;
    ReadFunctor<VehicleConfigMsgPayload> vehicleConfigMsg;
    ReadFunctor<RWSpeedMsgPayload> rwSpeedMsg;
    ReadFunctor<STAttMsgPayload> rateDataInMsg;

    Message<NavAttMsgPayload> navAttitudeOutputMsg;
    Message<FilterMsgPayload> inertialFilterOutputMsg;
    Message<FilterResidualsMsgPayload> attitudeResidualMsg;
    Message<FilterResidualsMsgPayload> rateResidualMsg;

   private:
    void customReset();
    void readFilterMeasurements();
    void writeOutputMessages(uint64_t currentSimNanos);
    void customInitializeUpdate();
    void customFinalizeUpdate();

    /*! Specific read messages */
    void readRWSpeedData();
    void readAttitudeData();
    void readRateData();
    void switchStateCovariance();

    RWArrayConfigMsgPayload rwArrayConfigPayload;

    SRukfInterface srukf{};
    double measNoiseScaling = 1;  //!< [-] Scale factor for the measurement noise
    //! measure up to MAX_ST_VEH_COUNT star trackers and one gyro
    xmera::measurement_queue<InertialAttitudeUkfMeasurementModel, MAX_ST_VEH_COUNT + 1> measurements = {};
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
    size_t numberOfStarTackers = 0;
    size_t measurementIndex = 0;
    double mrpSwitchThreshold = 1;  //!< [-] Threshold for switching MRP to/from the shadow set

    uint64_t previousSimNanos = 0;
};

#endif
