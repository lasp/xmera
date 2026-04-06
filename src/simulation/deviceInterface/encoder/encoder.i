// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module encoder
%{
   #include "encoder.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/utilities/simDefinitions.h>
%include <architecture/utilities/macroDefinitions.h>
%include "encoder.h"

%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
