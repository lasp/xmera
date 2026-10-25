// SPDX-License-Identifier: ISC
// Copyright (c) 2022, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef TABULAR_ATMOSPHERE_H
#define TABULAR_ATMOSPHERE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>

#include <simulation/environment/_GeneralModuleFiles/atmosphereBase.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

/*! @brief tabular atmosphere model */
class TabularAtmosphere : public AtmosphereBase {
private:
    // pulls density and temperature from atmospheric table at requested altitude, performs linear interpolation if
    // necessary
    void evaluateAtmosphereModel(AtmoPropsMsgPayload* msg, double currentTime);

    int altList_length;   // length of list of altitude values extracted from the atmosphere table
    int rhoList_length;   // length of list of density values extracted from the atmosphere table
    int tempList_length;  // length of list of temperature values extracted from the atmosphere table

    virtual void customreset(uint64_t CurrentClock);  // reset if error thrown

public:
    TabularAtmosphere();
    ~TabularAtmosphere();
    std::vector<double> altList;   //!< vector of doubles of altitude values extracted from the atmosphere table
    std::vector<double> rhoList;   //!< vector of doubles of density values extracted from the atmosphere table
    std::vector<double> tempList;  //!< vector of doubles of temperature values extracted from the atmosphere table
};

#endif /* TABULAR_ATMOSPHERE_H */
