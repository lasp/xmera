// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module opNavPoint
%{
   #include "opNavPoint.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "opNavPoint.h"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/CameraConfigMsgPayload.h>
%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavMsgPayload.h>
