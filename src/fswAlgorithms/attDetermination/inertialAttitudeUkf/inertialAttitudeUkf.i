// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module inertialAttitudeUkf
%{
   #include "inertialAttitudeUkf.h"
%}

%include <fswAlgorithms/_GeneralModuleFiles/srukfInterface.i>

%include "inertialAttitudeUkf.h"

%include <architecture/msgPayloadDef/FilterMsgPayload.h>
%include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>

%include <architecture/msgPayloadDef/STAttMsgPayload.h>
%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
%include <architecture/msgPayloadDef/IMUSensorBodyMsgPayload.h>
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
