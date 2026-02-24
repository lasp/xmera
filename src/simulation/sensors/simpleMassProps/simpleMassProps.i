// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module simpleMassProps
%{
   #include "simpleMassProps.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "simpleMassProps.h"

%include <architecture/msgPayloadDef/SCMassPropsMsgPayload.h>

%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
