#ifndef HINGEDRIGIDBODYMOTORSENSOR_H
#define HINGEDRIGIDBODYMOTORSENSOR_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/discretize.h>
#include <stdint.h>
#include <random>
/*! @brief Outputs measured angle and angle rate for a hinged rigid body, adding optional noise, bias, and
 * discretization.
 */
class HingedRigidBodyMotorSensor : public SysModel {
   public:
    HingedRigidBodyMotorSensor();
    ~HingedRigidBodyMotorSensor();

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);

    void setRNGSeed(unsigned int newSeed);  //!< for setting the seed

   public:
    double thetaNoiseStd;     //!< [rad] standard deviation for Gaussian noise to theta
    double thetaDotNoiseStd;  //!< [rad/s] standard deviation for Gaussian noise to theta dot
    double thetaBias;         //!< [rad] bias added to true theta
    double thetaDotBias;      //!< [rad/s] bias added to true theta dot
    double thetaLSB;          //!< [rad] discretization for theta
    double thetaDotLSB;       //!< [rad/s] discretization for theta dot

    ReadFunctor<HingedRigidBodyMsgPayload>
        hingedRigidBodyMotorSensorInMsg;  //!< input message for true rigid body state (theta, theta dot)

    Message<HingedRigidBodyMsgPayload>
        hingedRigidBodyMotorSensorOutMsg;  //!< output message for sensed rigid body state

    BSKLogger bskLogger;  //!< -- BSK Logging

   private:
    std::minstd_rand rGen;                  //!< -- Random number generator for model
    std::normal_distribution<double> rNum;  //!< -- Random number distribution for model
};

#endif
