#ifndef _SUN_SAFE_ACS_H_
#define _SUN_SAFE_ACS_H_

#include "../_GeneralModuleFiles/thrustGroupData.h"
#include "fswAlgorithms/effectorInterfaces/errorConversion/dvAttEffect.h"
#include <stdint.h>
#include <stdlib.h>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief module configuration message */
class SunSafeACS : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    ThrustGroupData thrData;                                 /*!< Collection of thruster configuration data*/
    ReadFunctor<CmdTorqueBodyMsgPayload> cmdTorqueBodyInMsg; /*!< -- The name of the Input message*/

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
