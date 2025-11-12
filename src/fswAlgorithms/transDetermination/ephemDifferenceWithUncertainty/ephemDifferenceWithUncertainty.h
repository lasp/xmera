#ifndef _EPHEM_DIFFERENCE_WITH_UNCERTAINTY_H_
#define _EPHEM_DIFFERENCE_WITH_UNCERTAINTY_H_

#include <architecture/messaging/messaging.h>
#include <architecture/utilities/eigenSupport.h>
#include <stdint.h>
#include <Eigen/Dense>

#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/FilterMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include "ephemDifferenceWithUncertaintyAlgorithm.h"

/*! @brief This module computes the difference between two ephemeris messages, and outputs the relative states into a
 * navigation and filter message */
class EphemDifferenceWithUncertainty : public SysModel {
   public:
    EphemDifferenceWithUncertainty() = default;
    ~EphemDifferenceWithUncertainty() = default;

    void updateState(uint64_t currentSimNanos) override;
    void reset(uint64_t currentSimNanos) override;

    void setCovarianceBase(const Eigen::MatrixXd stateCovariance);
    Eigen::MatrixXd getCovarianceBase() const;
    void setCovarianceSecondary(const Eigen::MatrixXd stateCovariance);
    Eigen::MatrixXd getCovarianceSecondary() const;

    ReadFunctor<EphemerisMsgPayload> ephemBaseInMsg;
    ReadFunctor<EphemerisMsgPayload> ephemSecondaryInMsg;
    Message<NavTransMsgPayload> navTransOutMsg;
    Message<FilterMsgPayload> filterOutMsg;

   private:
    EphemDifferenceWithUncertaintyAlgorithm algorithm{};
};

#endif
