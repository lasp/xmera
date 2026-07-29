// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef TEMPMEASUREMENT_H
#define TEMPMEASUREMENT_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/TemperatureMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/gauss_markov.h>

#include <Eigen/Dense>
#include <random>

typedef enum {
    TEMP_FAULT_STUCK_CURRENT, /*!< temp measurement is set to current value for all future time */
    TEMP_FAULT_STUCK_VALUE,   /*!< temp measurement is set to specified value for all future time */
    TEMP_FAULT_SPIKING,       /*!< temp measurement has a probability of spiking at each time step */
    TEMP_FAULT_NOMINAL
} TempFaultState_t;

/*! @brief Models a sensor to add noise, bias, and faults to temperature measurements.
 */
class TempMeasurement : public SysModel {
public:
    TempMeasurement();
    ~TempMeasurement();

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);

private:
    void applySensorErrors();

public:
    ReadFunctor<TemperatureMsgPayload> tempInMsg;  //!< True temperature measurement
    Message<TemperatureMsgPayload> tempOutMsg;     //!< Sensed temperature measurement
    BSKLogger bskLogger;                           //!< -- BSK Logging
    TempFaultState_t faultState;                   //!< [-] Fault status variable

    double senBias{};         //!< [-] Sensor bias value
    double senNoiseStd{};     //!< [-] Sensor noise value
    double walkBounds;        //!< [-] Gauss Markov walk bounds
    double stuckValue{};      //!< [C] Value for temp sensor to get stuck at
    double spikeProbability;  //!< [-] Probability of spiking at each time step (between 0 and 1)
    double spikeAmount;       //!< [-] Spike multiplier

private:
    double trueTemperature{};    //!< [C] Truth value for the temperature measurement
    double sensedTemperature{};  //!< [C] Temperature measurement as corrupted by noise and faults
    double pastValue{};          //!< [-] Measurement from last update (used only for faults)

    std::minstd_rand spikeProbabilityGenerator;  //! [-] Number generator for calculating probability of spike if faulty
    GaussMarkov<1> noiseModel;                   //! [-] Gauss Markov noise generation model
};

#endif
