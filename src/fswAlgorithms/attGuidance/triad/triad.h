// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _TRIAD_METHOD_
#define _TRIAD_METHOD_
#include <stdint.h>

#include <Eigen/Core>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/BodyHeadingMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/InertialHeadingMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

enum class CelestialBody { NotSun = 0, Sun = 1 };

enum class BodyAxisInput { inputBodyHeadingParameter = 0, inputBodyHeadingMsg = 1 };

enum class InertialAxisInput { inputInertialHeadingParameter = 0, inputInertialHeadingMsg = 1, inputEphemerisMsg = 2 };

/*! @brief Top level structure for the sub-module routines. */
class Triad : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    ReadFunctor<NavAttMsgPayload> attNavInMsg;                    //!< input msg measured attitude
    ReadFunctor<BodyHeadingMsgPayload> bodyHeadingInMsg;          //!< input body heading msg
    ReadFunctor<InertialHeadingMsgPayload> inertialHeadingInMsg;  //!< input inertial heading msg
    ReadFunctor<NavTransMsgPayload> transNavInMsg;                //!< input msg measured position
    ReadFunctor<EphemerisMsgPayload> ephemerisInMsg;              //!< input ephemeris msg
    Message<AttRefMsgPayload> attRefOutMsg;                       //!< output attitude reference message

    BSKLogger bskLogger{};                            //!< BSK Logging
    void setA1Hat_B(const Eigen::Vector3d& a1Hat_B);  //!< Setter for a1Hat_B variable
    const Eigen::Vector3d getA1Hat_B() const;         //!< Getter for a1Hat_B variable

    void setH1Hat_B(const Eigen::Vector3d& h1Hat_B);  //!< Setter for h1Hat_B variable
    const Eigen::Vector3d getH1Hat_B() const;         //!< Getter for h1Hat_B variable

    void setHHat_N(const Eigen::Vector3d& hHat_N);  //!< Setter for hHat_N variable
    const Eigen::Vector3d getHHat_N() const;        //!< Getter for hHat_N variable

    void setCelestialBodyInput(const CelestialBody& celestialBodyInput);  //!< Setter for celestialBodyInput variable
    const CelestialBody getCelestialBodyInput() const;                    //!< Getter for celestialBodyInput variable

    void setBodyAxisInput(const BodyAxisInput& bodyAxisInput);  //!< Setter for bodyAxisInput variable
    const BodyAxisInput getBodyAxisInput() const;               //!< Getter for bodyAxisInput variable

    void setInertialAxisInput(const InertialAxisInput& inertialAxisInput);  //!< Setter for inertialAxisInput variable
    const InertialAxisInput getInertialAxisInput() const;                   //!< Getter for inertialAxisInput variable

   private:
    /*! declare these quantities that always must be specified as flight software parameters */
    Eigen::Vector3d a1Hat_B;  //!< arrays axis direction in B frame (Solar array 1 body axis x+)

    /*! declare these optional quantities */
    Eigen::Vector3d h1Hat_B;  //!< main heading in B frame coordinates (HGA body Axis Y+)
    Eigen::Vector3d hHat_N;   //!< main heading in N frame coordinates (For SEP)
    CelestialBody celestialBodyInput;

    /*! declare these internal variables that are used by the module and should not be declared by the user */
    BodyAxisInput bodyAxisInput;          //!< flag variable to determine how the body axis input is specified
    InertialAxisInput inertialAxisInput;  //!< flag variable to determine how the inertial axis input is specified
};

#endif
