// SPDX-License-Identifier: ISC
// Copyright (c) 2020, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module mrpPD
%{
   #include "mrpPD.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include "mrpPD.h"

%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
