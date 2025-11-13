#ifndef ATTREFCORRECTION_H
#define ATTREFCORRECTION_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <Eigen/Core>
#include <stdint.h>

/*! @brief This module reads in the attitude reference message and adjusts it by a fixed rotation.  This allows a
 * general body-fixed frame B to align with this corrected reference frame.
 */
class AttRefCorrection : public SysModel {
   public:
    void reset(uint64_t callTime) final;
    void updateState(uint64_t callTime) final;

    void setSigmaRR0(const Eigen::Vector3d& sigma);
    const Eigen::Vector3d getSigmaRR0() const;

    ReadFunctor<AttRefMsgPayload> attRefInMsg;  //!< attitude reference input message
    Message<AttRefMsgPayload> attRefOutMsg;     //!< corrected attitude reference input message

   private:
    Eigen::Vector3d sigma_RR0{
        Eigen::Vector3d::Zero()};  //!< [-] current MRP attitude coordinate set with respect to the input reference
};

#endif
