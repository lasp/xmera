// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module planetNav
%{
    #include "planetNav.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>

// Expose the fixed-size Gauss-Markov matrices/vectors to Python (get/set as lists or
// numpy arrays, with wrong-size assignments rejected by a ValueError at the boundary).
EIGEN_MAT_WRAP(PlanetNavCovar, 159)
EIGEN_MAT_WRAP(PlanetNavErrorVector, 159)

%include "planetNav.h"

%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
