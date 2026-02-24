// SPDX-License-Identifier: ISC
// Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _SUN_SAFE_POINT_C_H_
#define _SUN_SAFE_POINT_C_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>

#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

/*! @brief Top level structure for the sun-safe attitude guidance routine.*/
class SunSafePoint_C : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    Message<AttGuidMsgPayload> attGuidanceOutMsg;    /*!< The name of the output message*/
    ReadFunctor<NavAttMsgPayload> sunDirectionInMsg; /*!< The name of the Input message*/
    ReadFunctor<NavAttMsgPayload> imuInMsg;          /*!< The name of the incoming IMU message*/
    double minUnitMag;                               /*!< -- The minimally acceptable norm of sun body vector*/
    double sunAngleErr;                              /*!< rad The current error between cmd and obs sun angle*/
    double smallAngle;      /*!< rad An angle value that specifies what is near 0 or 180 degrees */
    double eHat180_B[3];    /*!< -- Eigen axis to use if commanded axis is 180 from sun axis */
    double sunMnvrVec[3];   /*!< -- The eigen axis that we want to rotate on to get sun*/
    double sHatBdyCmd[3];   /*!< -- Desired body vector to point at the sun*/
    double omega_RN_B[3];   /*!< -- Desired body rate vector if no sun direction is available */
    double sunAxisSpinRate; /*!< r/s Desired constant spin rate about sun heading vector */

    AttGuidMsgPayload attGuidanceOutBuffer; /*!< -- The output data that we compute*/
    BSKLogger bskLogger = {};               //!< BSK Logging
};

#endif
