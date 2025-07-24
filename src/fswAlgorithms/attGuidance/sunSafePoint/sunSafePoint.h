/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef _BASILISK_SUN_SAFE_POINT_H_
#define _BASILISK_SUN_SAFE_POINT_H_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "fswAlgorithms/attGuidance/sunSafePoint/sunSafePointAlgorithm.h"
#include <stdint.h>
#include <Eigen/Dense>

/*! @brief Sun safe point attitude guidance class. */
class SunSafePoint : public SysModel {
   public:
    SunSafePoint() = default;   //!< Constructor
    ~SunSafePoint() = default;  //!< Destructor

    void reset(uint64_t currentSimNanos) override;        //!< Reset member function
    void updateState(uint64_t currentSimNanos) override;  //!< Update member function

    double getMinUnitMag() const;       //!< Getter method for the minimally accepted sun body vector norm
    double getSmallAngle() const;       //!< Getter method for the small alignment tolerance angle near 0 or 180 degrees
    double getSunAxisSpinRate() const;  //!< Getter method for the desired constant spin rate about sun heading vector
    Eigen::Vector3d getOmega_RN_B()
        const;  //!< Getter method for the desired body rate vector if no sun direction is available
    void setMinUnitMag(const double minUnitMag);   //!< Setter method for the minimally accepted sun body vector norm
    Eigen::Vector3d getSHatBdyCmd() const;        //!< Getter method for the desired body vector to point at the sun
    void setSmallAngle(
        const double smallAngle);  //!< Setter method for the small alignment tolerance angle near 0 or 180 degrees
    void setSunAxisSpinRate(
        const double sunAxisSpinRate);  //!< Setter method for the desired constant spin rate about sun heading vector
    void setOmega_RN_B(const Eigen::Vector3d& omega_RN_B);  //!< Setter method for the desired body rate vector if no
                                                            //!< sun direction is available
    void setSHatBdyCmd(Eigen::Vector3d& sHatBdyCmd);  //!< Setter method for the desired body vector to point at the sun

    ReadFunctor<NavAttMsgPayload> imuInMsg;           //!< IMU attitude guidance input message
    ReadFunctor<NavAttMsgPayload> sunDirectionInMsg;  //!< Sun attitude guidance input message
    Message<AttGuidMsgPayload> attGuidanceOutMsg;     //!< Attitude guidance output message

    BSKLogger* bskLogger;  //!< BSK Logging

   private:
    SunSafePointAlgorithm algorithm;  //!< Algorithm for sunSafePoint guidance logic (BSK-agnostic)
};

#endif
