// SPDX-License-Identifier: ISC
// Copyright (c) 2019, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module magnetometer
%{
   #include "magnetometer.h"
%}

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "magnetometer.h"

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/MagneticFieldMsgPayload.h>

%include <architecture/msgPayloadDef/TAMSensorMsgPayload.h>
