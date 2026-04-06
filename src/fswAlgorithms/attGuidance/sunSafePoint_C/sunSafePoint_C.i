// SPDX-License-Identifier: ISC
// Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module sunSafePoint_C
%{
   #include "sunSafePoint_C.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "sunSafePoint_C.h"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
