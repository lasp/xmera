// SPDX-License-Identifier: ISC
// Copyright (c) 2019, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module tamComm
%{
   #include "tamComm.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "tamComm.h"

%include <architecture/msgPayloadDef/TAMSensorBodyMsgPayload.h>
%include <architecture/msgPayloadDef/TAMSensorMsgPayload.h>
