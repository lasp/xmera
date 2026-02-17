// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _GEODETIC_CONV_H_
#define _GEODETIC_CONV_H_

#include <math.h>
#include <Eigen/Dense>

/*! @brief Collection of utility functions for converting in/out of planet-centric reference frames.

The geodeticConversion library contains simple transformations between inertial coordinates and planet-fixed coordinates
in a general way.

No support is provided for non-spherical bodies. Transformations are scripted from Vallado.

 */

Eigen::Vector3d PCI2PCPF(Eigen::Vector3d pciPosition, double J20002Pfix[3][3]);
Eigen::Vector3d PCPF2LLA(Eigen::Vector3d pciPosition, double planetEqRadius, double planetPoRad = -1.0);
Eigen::Vector3d PCI2LLA(Eigen::Vector3d pciPosition,
                        double J20002Pfix[3][3],
                        double planetEqRad,
                        double planetPoRad = -1.0);
Eigen::Vector3d LLA2PCPF(Eigen::Vector3d llaPosition, double planetEqRad, double planetPoRad = -1.0);
Eigen::Vector3d PCPF2PCI(Eigen::Vector3d pcpfPosition, double J20002Pfix[3][3]);
Eigen::Vector3d LLA2PCI(Eigen::Vector3d llaPosition,
                        double J20002Pfix[3][3],
                        double planetEqRad,
                        double planetPoRad = -1.0);
Eigen::Matrix3d C_PCPF2SEZ(double lat, double longitude);

#endif
