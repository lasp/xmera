// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _ONE_AXIS_SOLAR_ARRAY_POINT_
#define _ONE_AXIS_SOLAR_ARRAY_POINT_

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/BodyHeadingMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/InertialHeadingMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

typedef enum celestialBody { notSun = 0, Sun = 1 } CelestialBody;

typedef enum alignmentPriority { prioritizeAxisAlignment = 0, prioritizeSolarArrayAlignment = 1 } AlignmentPriority;

typedef enum bodyAxisInput { inputBodyHeadingParameter = 0, inputBodyHeadingMsg = 1 } BodyAxisInput;

typedef enum inertialAxisInput {
    inputInertialHeadingParameter = 0,
    inputInertialHeadingMsg = 1,
    inputEphemerisMsg = 2
} InertialAxisInput;

typedef enum refFrameSolution { determinate = 0, indeterminate = 1 } RefFrameSolution;

/*! @brief Top level structure for the sub-module routines. */
class OneAxisSolarArrayPoint : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /*! declare these quantities that always must be specified as flight software parameters */
    double a1Hat_BInput[3];               //!< arrays axis direction in B frame
    AlignmentPriority alignmentPriority;  //!< flag to indicate which constraint must be prioritized

    /*! declare these optional quantities */
    double h1Hat_BInput[3];  //!< main heading in B frame coordinates
    double h2Hat_BInput[3];  //!< secondary heading in B frame coordinates
    double hHat_NInput[3];   //!< main heading in N frame coordinates
    double a2Hat_BInput[3];  //!< body frame heading that should remain as close as possible to Sun heading
    CelestialBody celestialBodyInput;

    /*! declare these internal variables that are used by the module and should not be declared by the user */
    BodyAxisInput bodyAxisInput;                //!< flag variable to determine how the body axis input is specified
    InertialAxisInput inertialAxisInput;        //!< flag variable to determine how the inertial axis input is specified
    ReadFunctor<NavAttMsgPayload> attNavInMsg;  //!< input msg measured attitude
    ReadFunctor<BodyHeadingMsgPayload> bodyHeadingInMsg;          //!< input body heading msg
    ReadFunctor<InertialHeadingMsgPayload> inertialHeadingInMsg;  //!< input inertial heading msg
    ReadFunctor<NavTransMsgPayload> transNavInMsg;                //!< input msg measured position
    ReadFunctor<EphemerisMsgPayload> ephemerisInMsg;              //!< input ephemeris msg
    Message<AttRefMsgPayload> attRefOutMsg;                       //!< output attitude reference message

    BSKLogger bskLogger{};  //!< BSK Logging
};

#endif
