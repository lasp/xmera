// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module etSphericalControl
%{
   #include "etSphericalControl.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "etSphericalControl.h"

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/CmdForceInertialMsgPayload.h>
%include <architecture/msgPayloadDef/CmdForceBodyMsgPayload.h>
