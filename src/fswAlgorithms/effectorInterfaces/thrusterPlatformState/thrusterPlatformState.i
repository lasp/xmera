// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module thrusterPlatformState
%{
   #include "thrusterPlatformState.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "thrusterPlatformState.h"

%include <architecture/msgPayloadDef/THRConfigMsgPayload.h>
%include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
