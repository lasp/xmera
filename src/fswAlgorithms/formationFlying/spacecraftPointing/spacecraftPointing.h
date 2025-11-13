#ifndef _SPACECRAFTPOINTING_H_
#define _SPACECRAFTPOINTING_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>

#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

/*! @brief Top level structure for the spacecraft pointing module.*/
class SpacecraftPointing : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    Message<AttRefMsgPayload> attReferenceOutMsg;        /*!< The name of the output message */
    ReadFunctor<NavTransMsgPayload> chiefPositionInMsg;  /*!< The name of the Input message of the chief */
    ReadFunctor<NavTransMsgPayload> deputyPositionInMsg; /*!< The name of the Input message of the deputy */

    double alignmentVector_B[3]; /*!< Vector within the B-frame that points to antenna */
    double sigma_BA[3];          /*!< -- MRP of B-frame with respect to A-frame */
    double old_sigma_RN[3];      /*!< -- MRP of previous timestep */
    double old_omega_RN_N[3];    /*!< -- Omega of previous timestep */
    int i;                       /*!< -- Flag used to set incorrect numerical answers to zero */
    uint64_t priorTime;          /*!< [ns] Last time the attitude control is called */

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
