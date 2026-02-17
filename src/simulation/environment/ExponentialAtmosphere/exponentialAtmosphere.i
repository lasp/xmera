// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module exponentialAtmosphere
%{
    #include "exponentialAtmosphere.h"
%}

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>
%include <std_string.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/environment/_GeneralModuleFiles/atmosphereBase.h>
%include "exponentialAtmosphere.h"

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/AtmoPropsMsgPayload.h>
