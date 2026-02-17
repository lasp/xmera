// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module stComm
%{
   #include "stComm.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "stComm.h"

%include <architecture/msgPayloadDef/STSensorMsgPayload.h>
%include <architecture/msgPayloadDef/STAttMsgPayload.h>
