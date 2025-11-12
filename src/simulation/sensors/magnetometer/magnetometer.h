#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <random>
#include <vector>

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/MagneticFieldMsgPayload.h>
#include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
#include <architecture/msgPayloadDef/TAMSensorMsgPayload.h>

#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/gauss_markov.h>
#include <architecture/utilities/saturate.h>
#include <Eigen/Dense>

/*! @brief magnetometer class */
class Magnetometer : public SysModel {
   public:
    Magnetometer();
    ~Magnetometer();
    void reset(uint64_t CurrentClock);           //!< Method for reseting the module
    void updateState(uint64_t currentSimNanos);  //!< Method to update state for runtime
    void readInputMessages();                    //!< Method to read the input messages
    void computeTrueOutput();                    //!< Method to compute the true magnetic field vector
    void computeMagData();                       //!< Method to get the magnetic field vector information
    void applySensorErrors();                    //!< Method to set the actual output of the sensor with errors
    void applySaturation();                      //!< Apply saturation effects to sensed output (floor and ceiling)
    void writeOutputMessages(uint64_t Clock);    //!< Method to write the output message to the system
    Eigen::Matrix3d setBodyToSensorDCM(double yaw,
                                       double pitch,
                                       double roll);  //!< Utility method to configure the sensor DCM

   public:
    ReadFunctor<SCStatesMsgPayload> stateInMsg;     //!< [-] input message for spacecraft states
    ReadFunctor<MagneticFieldMsgPayload> magInMsg;  //!< [-] input message for magnetic field data in inertial frame N
    Message<TAMSensorMsgPayload> tamDataOutMsg;     //!< [-] Message for TAM output data in sensor frame S
    Eigen::Matrix3d dcm_SB;                         //!< [-] DCM from body frame to sensor frame
    Eigen::Vector3d tam_S;                          //!< [T] Magnetic field vector in sensor frame
    Eigen::Vector3d tamSensed_S;                    //!< [T] Measurement including perturbations
    Eigen::Vector3d tamTrue_S;                      //!< [T] Measurement without perturbations
    double scaleFactor;                             //!< [-] Scale factor applied to sensor
    Eigen::Vector3d senBias;                        //!< [T] Sensor bias vector
    Eigen::Vector3d senNoiseStd;                    //!< [T] Sensor noise standard deviation vector

    Eigen::Vector3d walkBounds;  //!< [T] "3-sigma" errors to permit for states
    double maxOutput;            //!< [T] Maximum output for saturation application
    double minOutput;            //!< [T] Minimum output for saturation application
    BSKLogger bskLogger;         //!< -- BSK Logging

   private:
    MagneticFieldMsgPayload magData;  //!< [-] Magnetic field in inertial N frame
    SCStatesMsgPayload stateCurrent;  //!< [-] Current spacecraft state
    uint64_t numStates;               //!< [-] Number of States for Gauss Markov Models
    GaussMarkov noiseModel;           //!< [-] Gauss Markov noise generation model
    Saturate saturateUtility;         //!< [-] Saturation utility
};

#endif
