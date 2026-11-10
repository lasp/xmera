// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef EXPONENTIAL_ATMOSPHERE_H
#define EXPONENTIAL_ATMOSPHERE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>

#include <simulation/environment/_GeneralModuleFiles/atmosphereBase.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

/*! @brief exponential atmosphere model */
class ExponentialAtmosphere : public AtmosphereBase {
public:
    ExponentialAtmosphere();
    ~ExponentialAtmosphere();

private:
    void evaluateAtmosphereModel(AtmoPropsMsgPayload* msg, double currentTime);

public:
    double baseDensity;        //!< [kg/m^3] Density at h=0
    double scaleHeight;        //!< [m] Exponential characteristic height
    double localTemp = 293.0;  //!< [K] Local atmospheric temperature; set to be constant.
};

#endif /* EXPONENTIAL_ATMOSPHERE_H */
