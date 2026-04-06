// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module imuComm
%{
   #include "imuComm.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "imuComm.h"

%include <architecture/msgPayloadDef/IMUSensorBodyMsgPayload.h>
%include <architecture/msgPayloadDef/IMUSensorMsgPayload.h>
