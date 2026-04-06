// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module spacecraftLocation
%{
    #include "spacecraftLocation.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include "spacecraftLocation.h"

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/AccessMsgPayload.h>
