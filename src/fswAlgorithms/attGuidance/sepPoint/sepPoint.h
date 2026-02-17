// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _SEP_POINT_
#define _SEP_POINT_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/BodyHeadingMsgPayload.h>
#include <architecture/msgPayloadDef/InertialHeadingMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <fswAlgorithms/attGuidance/_GeneralModuleFiles/constrainedAxisPointingLibrary.h>

/*! @brief A class to perform EMA SEP pointing */
class SepPoint : public SysModel {
   public:
    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);

    AlignmentPriority alignmentPriority;  //!< flag to indicate which flyby model is being used
    double a1Hat_B[3];                    //!< solar array drive axis in B-frame coordinates
    double a2Hat_B[3];                    //!< Sun-constrained axis in B-frame coordinates
    double beta;                          //!< Sun-constraint half-cone angle

    ReadFunctor<NavAttMsgPayload> attNavInMsg;                    //!< input msg measured attitude
    ReadFunctor<BodyHeadingMsgPayload> bodyHeadingInMsg;          //!< input body heading msg
    ReadFunctor<InertialHeadingMsgPayload> inertialHeadingInMsg;  //!< input inertial heading msg
    Message<AttRefMsgPayload> attRefOutMsg;                       //!< Attitude reference output message

   private:
    int callCount;           //!< count variable used in the finite difference logic
    uint64_t T1NanoSeconds;  //!< callTime one update step prior
    uint64_t T2NanoSeconds;  //!< callTime two update steps prior
    double sigma_RN_1[3];    //!< reference attitude one update step prior
    double sigma_RN_2[3];    //!< reference attitude two update steps prior
    BSKLogger bskLogger;     //!< BSK Logging
};

#endif
