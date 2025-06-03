/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef _THRUSTER_FORCE_MAPPING_ALGORITHM_H
#define _THRUSTER_FORCE_MAPPING_ALGORITHM_H

#include <stdint.h>

#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayCmdForceMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
#include "architecture/utilities/macroDefinitions.h"
#include <Eigen/Dense>

typedef Eigen::Vector<double, MAX_EFF_CNT> Vector36d;

enum class ThrForceSign { POSITIVE = +1, NEGATIVE = -1 };

/*!@brief Thruster force mapping algorithm class. */
class ThrForceMappingAlgorithm {
   public:
    ThrForceMappingAlgorithm() = default;                                     //!< Constructor
    ~ThrForceMappingAlgorithm() = default;                                    //!< Destructor
    void reset(uint64_t callTime, THRArrayConfigMsgPayload& thrConfigInMsg);  //!< Reset method
    THRArrayCmdForceMsgPayload update(uint64_t callTime,
                                      CmdTorqueBodyMsgPayload& cmdTorqueInMsg,
                                      VehicleConfigMsgPayload& vehConfigInMsg);  //!< Update method

    Eigen::Matrix3d getControlAxesB() const;            //!< Getter method for thruster control axes
    void setControlAxesB(const Eigen::Matrix3d& axes);  //!< Setter method for thruster control axes
    Vector36d getThrForceMag() const;                   //!< Getter method for the thruster force magnitude
    void setThrForceMag(const Vector36d& forceMag);     //!< Setter method for the thruster force magnitude
    ThrForceSign getThrForceSign() const;               //!< Getter method for the thruster force sign
    void setThrForceSign(ThrForceSign sign);            //!< Setter method for the thruster force sign
    double getAngErrThresh() const;                     //!< Getter method for the angular error threshold
    void setAngErrThresh(double thresh);                //!< Setter method for the angular error threshold
    double getEpsilon() const;                          //!< Getter method for the epsilon value
    void setEpsilon(double eps);                        //!< Setter method for the epsilon value
    bool getUse2ndLoop() const;  //!< Getter method for whether the second least squares fitting loop should be used
    void setUse2ndLoop(
        bool loopFlag);  //!< Getter method for whether the second least squares fitting loop should be used

   private:
    Vector36d findMinimumNormForce(const Eigen::Matrix<double, 3, MAX_EFF_CNT>& D,
                                   const Eigen::Vector3d& Lr_B,
                                   uint32_t numForces) const;

    Eigen::Matrix3d controlAxes_B{};  //!< [] array of the control unit axes
    Vector36d thrForceMag{};          //!< vector of thruster force magnitudes
    ThrForceSign thrForceSign =
        ThrForceSign::POSITIVE;  //!< [] Flag indicating if positive or negative thruster solutions are found
    double angErrThresh{};       //!< [r] Angular error at which thruster forces are scaled to not be super-saturated
    double epsilon{};            //!< variable specifying what is considered a small number
    bool use2ndLoop{};  //!< [] flag indicating if the 2nd least squares fitting loop should be used (1) or not used (0
                        //!< - default)
    uint32_t numControlAxes{};  //!< [] counter indicating how many orthogonal axes are controlled
    uint32_t numThrusters{};    //!< [] The number of thrusters available on vehicle
    double outTorqAngErr{};     //!< [r] Angular error of effector torque
    Eigen::Matrix<double, MAX_EFF_CNT, 3> rThruster_B{};  //!< [m] local copy of the thruster locations
    Eigen::Matrix<double, MAX_EFF_CNT, 3>
        gtThruster_B{};  //!< [] local copy of the thruster force unit direction vectors
};

#endif
