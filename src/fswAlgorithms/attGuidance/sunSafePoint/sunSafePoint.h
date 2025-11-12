#ifndef _XMERA_SUN_SAFE_POINT_H_
#define _XMERA_SUN_SAFE_POINT_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include "sunSafePointAlgorithm.h"
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
    Eigen::Vector3d getSHatBdyCmd() const;        //!< Getter method for the desired body vector to point at the sun
    void setMinUnitMag(const double minUnitMag);  //!< Setter method for the minimally accepted sun body vector norm
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
