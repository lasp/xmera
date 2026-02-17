// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef EXT_PULSED_TORQUE_H
#define EXT_PULSED_TORQUE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
#include <architecture/utilities/bskLogging.h>

/*! @brief external pulsed torque module class */
class ExtPulsedTorque : public SysModel, public DynamicEffector {
   public:
    ExtPulsedTorque();
    ~ExtPulsedTorque();

    void updateState(uint64_t currentSimNanos);
    void linkInStates(DynParamManager& statesIn);
    void writeOutputMessages(uint64_t currentClock);
    void readInputMessages();
    void computeForceTorque(double integTime, double timeStep);

   private:
    int c;  //!< numer of time steps for pulse

   public:
    Eigen::Vector3d pulsedTorqueExternalPntB_B;  //!< pulsed torque vector about point B, in B frame components
    int countOnPulse;                            //!< number of integration time steps to simulate a pulse
    int countOff;                                //!< number of integration time steps to have no pulses
    BSKLogger bskLogger;                         //!< -- BSK Logging
};

#endif
