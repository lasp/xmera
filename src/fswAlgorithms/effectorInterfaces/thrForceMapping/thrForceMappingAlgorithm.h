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

class ThrForceMappingAlgorithm {
   public:
    void reset(uint64_t callTime, THRArrayConfigMsgPayload& thrConfigInMsg);
    THRArrayCmdForceMsgPayload update(uint64_t callTime,
                                      CmdTorqueBodyMsgPayload& cmdTorqueInMsg,
                                      VehicleConfigMsgPayload& vehConfigInMsg);

    Eigen::Matrix3d getControlAxesB() const;
    void setControlAxesB(const Eigen::Matrix3d& axes);
    Vector36d getThrForceMag() const;
    void setThrForceMag(const Vector36d& forceMag);
    ThrForceSign getThrForceSign() const;
    void setThrForceSign(ThrForceSign sign);
    double getAngErrThresh() const;
    void setAngErrThresh(double thresh);
    double getEpsilon() const;
    void setEpsilon(double eps);
    bool getUse2ndLoop() const;
    void setUse2ndLoop(bool loopFlag);

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
