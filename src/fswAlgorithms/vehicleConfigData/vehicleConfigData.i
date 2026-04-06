// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module vehicleConfigData
%{
   #include "vehicleConfigData.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include "vehicleConfigData.h"

%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
