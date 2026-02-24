// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module planetEphemeris
%{
   #include "planetEphemeris.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>

namespace std {
    %template(classicElementVector) vector<ClassicElements>;
    %template() vector<string, allocator<string>>;
    %template(DoubleVector) vector<double, allocator<double>>;
}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "planetEphemeris.h"
%include <architecture/utilities/orbitalMotion.h>
%include <architecture/utilities/astroConstants.h>


%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>
