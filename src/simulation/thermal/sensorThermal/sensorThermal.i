// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module sensorThermal
%{
    #include "sensorThermal.h"
%}

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include "sensorThermal.h"

%include <architecture/msgPayloadDef/TemperatureMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/EclipseMsgPayload.h>

%include <architecture/msgPayloadDef/DeviceStatusMsgPayload.h>
