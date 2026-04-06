// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module rwConfigData
%{
   #include "rwConfigData.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "rwConfigData.h"

%include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/RWConstellationMsgPayload.h>
