// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef CENTERED_DIPOLE_MAGNETIC_FIELD_H
#define CENTERED_DIPOLE_MAGNETIC_FIELD_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <architecture/_GeneralModuleFiles/sys_model.h>

#include <simulation/environment/_GeneralModuleFiles/magneticFieldBase.h>

#include <architecture/utilities/bskLogging.h>

/*! @brief magnetic field centered dipole class */
class MagneticFieldCenteredDipole : public MagneticFieldBase {
   public:
    MagneticFieldCenteredDipole();
    ~MagneticFieldCenteredDipole();

   private:
    void evaluateMagneticFieldModel(MagneticFieldMsgPayload* msg, double currentTime);

   public:
    // More info on these IGRF parameters can be found on this [link](https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html)
    double g10;  //!< [T] IGRF coefficient g_1^0
    double g11;  //!< [T] IGRF coefficient g_1^1
    double h11;  //!< [T] IGRF coefficient h_1^1

    BSKLogger bskLogger;  //!< -- BSK Logging
};

#endif /* CENTERED_DIPOLE_MAGNETIC_FIELD_H */
