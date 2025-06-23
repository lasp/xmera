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

#ifndef _SUN_SAFE_POINT_ALGORITHM_H_
#define _SUN_SAFE_POINT_ALGORITHM_H_

#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include <stdint.h>
#include <Eigen/Dense>

/*! @brief Sun safe point attitude guidance class. */
class SunSafePointAlgorithm {
   public:
    SunSafePointAlgorithm() = default;   //!< Constructor
    ~SunSafePointAlgorithm() = default;  //!< Destructor

    void reset(uint64_t currentSimNanos);  //!< Reset member function
    AttGuidMsgPayload update(uint64_t currentSimNanos,
                             NavAttMsgPayload imuInMsg,
                             NavAttMsgPayload sunDirectionInMsg);  //!< Update member function

    double getMinUnitMag() const;       //!< Getter method for the minimally accepted sun body vector norm
    double getSmallAngle() const;       //!< Getter method for the small alignment tolerance angle near 0 or 180 degrees
    double getSunAxisSpinRate() const;  //!< Getter method for the desired constant spin rate about sun heading vector
    const Eigen::Vector3d &getOmega_RN_B()
        const;  //!< Getter method for the desired body rate vector if no sun direction is available
    const Eigen::Vector3d &getSHatBdyCmd() const;  //!< Getter method for the desired body vector to point at the sun
    void setMinUnitMag(const double minUnitMag);   //!< Setter method for the minimally accepted sun body vector norm
    void setSmallAngle(
        const double smallAngle);  //!< Setter method for the small alignment tolerance angle near 0 or 180 degrees
    void setSunAxisSpinRate(
        const double sunAxisSpinRate);  //!< Setter method for the desired constant spin rate about sun heading vector
    void setOmega_RN_B(const Eigen::Vector3d &omega_RN_B);  //!< Setter method for the desired body rate vector if no
                                                            //!< sun direction is available
    void setSHatBdyCmd(Eigen::Vector3d &sHatBdyCmd);  //!< Setter method for the desired body vector to point at the sun

   private:
    void computeAttGuidanceStates(double sHatNorm);  //!< Method for computing the attitude guidance states sigma_BR and
                                                     //!< omega_RN_B if a valid sun direction vector is available
    void computeHubAngularRateError(
        NavAttMsgPayload imuInMsg);  //!< Method for computing the hub angular rate error omega_BR_B
    bool sunDirectionIsAvailable(
        double sHatNorm) const;  //!< Method for determining if a valid sun direction vector is available

    double minUnitMag;           //!< The minimally acceptable norm of sun body vector
    double smallAngle;           //!< [rad] An angle value that specifies what is near 0 or 180 degrees
    double sunAxisSpinRate;      //!< [rad/s] Desired constant spin rate about sun heading vector
    double sunAngleErr;          //!< [rad] The current error between cmd and obs sun angle
    Eigen::Vector3d omega_RN_B;  //!< [rad/s] Desired body rate vector if no sun direction is available
    Eigen::Vector3d sHatBdyCmd;  //!< Desired body vector to point at the sun
    Eigen::Vector3d eHat180_B;   //!< Eigen axis to use if commanded axis is 180 from sun axis
    Eigen::Vector3d sunMnvrVec;  //!< The Eigen axis that we want to rotate on to get sun

    AttGuidMsgPayload attGuidanceOutBuffer;  //!< Attitude guidance output message buffer
    NavAttMsgPayload sunDirectionInBuffer;   //!< Sun attitude guidance input message buffer
};

#endif
