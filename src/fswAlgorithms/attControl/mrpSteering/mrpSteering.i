// SPDX-License-Identifier: ISC
// Copyright (c) 2020, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module mrpSteering
%{
   #include "mrpSteering.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "mrpSteering.h"
%include "mrpSteeringAlgorithm.h"

%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
%include <architecture/msgPayloadDef/RateCmdMsgPayload.h>
