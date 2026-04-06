// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module thrusterPlatformReference
%{
   #include "thrusterPlatformReference.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "thrusterPlatformReference.h"

%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/THRConfigMsgPayload.h>
%include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
%include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
%include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
%include <architecture/msgPayloadDef/BodyHeadingMsgPayload.h>
