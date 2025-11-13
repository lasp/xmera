#ifndef _SUN_SAFE_POINT_ALGORITHM_H_
#define _SUN_SAFE_POINT_ALGORITHM_H_

#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <stdint.h>
#include <Eigen/Dense>

/*! @brief Sun safe point attitude guidance algorithm class. */
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
    Eigen::Vector3d getOmega_RN_B()
        const;  //!< Getter method for the desired body rate vector if no sun direction is available
    Eigen::Vector3d getSHatBdyCmd() const;  //!< Getter method for the desired body vector to point at the sun
    void setMinUnitMag(double minUnitMag);  //!< Setter method for the minimally accepted sun body vector norm
    void setSmallAngle(
        double smallAngle);  //!< Setter method for the small alignment tolerance angle near 0 or 180 degrees
    void setSunAxisSpinRate(
        const double sunAxisSpinRate);  //!< Setter method for the desired constant spin rate about sun heading vector
    void setOmega_RN_B(const Eigen::Vector3d& omega_RN_B);  //!< Setter method for the desired body rate vector if no
                                                            //!< sun direction is available
    void setSHatBdyCmd(Eigen::Vector3d& sHatBdyCmd);  //!< Setter method for the desired body vector to point at the sun

   private:
    void computeAttGuidanceStates(double sHatNorm);  //!< Method for computing the attitude guidance states sigma_BR and
                                                     //!< omega_RN_B if a valid sun direction vector is available
    void computeHubAngularRateError(
        NavAttMsgPayload imuInMsg);  //!< Method for computing the hub angular rate error omega_BR_B
    bool sunDirectionIsAvailable(
        const double sHatNorm) const;  //!< Method for determining if a valid sun direction vector is available

    double minUnitMag{0.1};        //!< The minimally acceptable norm of sun body vector (Must be positive)
    double smallAngle{};           //!< [rad] An angle value that specifies what is near 0 or 180 degrees (Must be >= 0)
    double sunAxisSpinRate{};      //!< [rad/s] Desired constant spin rate about sun heading vector
    Eigen::Vector3d omega_RN_B{};  //!< [rad/s] Desired body rate vector if no sun direction is available
    Eigen::Vector3d sHatBdyCmd{0.0, 0.0, 1.0};  //!< Desired body vector to point at the sun
    Eigen::Vector3d eHat180_B{1.0, 0.0, 0.0};   //!< Eigen axis to use if commanded axis is 180 from sun axis

    AttGuidMsgPayload attGuidanceOutBuffer;  //!< Attitude guidance output message buffer
    NavAttMsgPayload sunDirectionInBuffer;   //!< Sun attitude guidance input message buffer
};

#endif
