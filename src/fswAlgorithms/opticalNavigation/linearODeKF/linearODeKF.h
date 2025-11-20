// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

/*! @brief Top level structure for the flyby OD linear kalman filter.
 Used to estimate the spacecraft's inertial position relative to a body.
 */

#ifndef LINEAR_OD_EKF_H
#define LINEAR_OD_EKF_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/FilterMsgPayload.h>
#include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/msgPayloadDef/OpNavUnitVecMsgPayload.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/orbitalMotion.h>

#include <fswAlgorithms/_GeneralModuleFiles/ekfInterface.h>

#ifndef SWIG
struct LinearODeKFMeasurementModel : public EkfMeasurementModel<3, 3> {
public:
    //! [-] observation measurement model
    VectorMd model(const EkfStateVector<3>& state) const override {
        return state.position / state.position.norm();
    }

    /*! Compute the measurement matrix to linearize the measurement model
        @param FilterStateVector state
        @return Eigen::MatrixXd
    */
    MatrixMMd measurementMatrix(const EkfStateVector<3>& state) const override {
        auto const& position = state.position;

        MatrixMMd measurementMatrix = MatrixMMd::Zero(position.size(), state.size());
        measurementMatrix.block(0, 0, position.size(), position.size()) =
            ( 1 / position.norm()
            * ( MatrixMMd::Identity(position.size(), position.size())
              - 1 / pow(position.norm(), 2) * position * position.transpose()
              )
            );

        return measurementMatrix;
    }

    VectorMd subtract(const VectorMd& observed, const VectorMd& predicted) const override {
        return observed - predicted;
    }

    VectorMd getObservation() const override {
        return this->observation;
    }

    MatrixMMd getNoise() const override {
        return this->measNoise;
    }

    void setPreFitResiduals(VectorMd const& preFitResiduals) override {
        this->preFitResiduals = preFitResiduals;
    }

    void setPostFitResiduals(VectorMd const& postFitResiduals) override {
        this->postFitResiduals = postFitResiduals;
    }

public:
    VectorMd observation = {};       //!< [-] Observation data vector
    MatrixMMd measNoise   = {};       //!< [-] Measurement noise

public:
    VectorMd preFitResiduals  = {};  //!< [-] Observation pre fit residuals
    VectorMd postFitResiduals = {};  //!< [-] Observation post fit residuals
};
#endif

class LinearODeKF : public SysModel {
   public:
    explicit LinearODeKF(FilterType type){};
    ~LinearODeKF() = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    void setMeasurementNoiseScale(double measurementNoiseScale);
    double getMeasurementNoiseScale() const;

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

   private:
    EkfInterface<3, 3> ekf{FilterType::Classical};
    double measNoiseScaling = 1;  //!< [-] Scale factor for the measurement noise
    xmera::measurement_queue<LinearODeKFMeasurementModel, 1> measurements = {};  //!< [Measurements] All
    uint64_t previousSimNanos = 0;
};

#endif
