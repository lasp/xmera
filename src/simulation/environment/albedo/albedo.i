// SPDX-License-Identifier: ISC
// Copyright (c) 2020, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module albedo
%{
   #include "albedo.h"
%}

%include <std_string.i>
%include <std_vector.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%import <architecture/_GeneralModuleFiles/sys_model.i>
%include "albedo.h"

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/AlbedoMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
