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
#include <fswAlgorithms/_GeneralModuleFiles/measurementModels.h>

class LinearODeKF : public EkfInterface {
   public:
    LinearODeKF(FilterType type) : EkfInterface(type) {};
    ~LinearODeKF() = default;

   private:
    void customreset() final;
    void readFilterMeasurements() final;
    void writeOutputMessages(uint64_t currentSimNanos) final;
    static Eigen::MatrixXd measurementMatrix(const FilterStateVector& state);

   public:
    ReadFunctor<OpNavUnitVecMsgPayload> opNavHeadingMsg;
    OpNavUnitVecMsgPayload opNavHeadingBuffer;
    Message<NavTransMsgPayload> navTransOutMsg;
    Message<FilterMsgPayload> opNavFilterMsg;
    Message<FilterResidualsMsgPayload> opNavResidualMsg;

    void setConstantVelocity(const Eigen::Vector3d& velocity);
    std::optional<Eigen::Vector3d> getConstantVelocity() const;

   private:
    std::optional<Eigen::Vector3d> constantVelocity;         //!< Unestimated constant velocity
    std::optional<Eigen::Vector3d> constantVelocityInitial;  //!< Initial value of constant velocity
};

#endif
