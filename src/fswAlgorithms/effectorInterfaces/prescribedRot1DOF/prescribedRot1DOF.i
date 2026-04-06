// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module prescribedRot1DOF
%{
   #include "prescribedRot1DOF.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "prescribedRot1DOF.h"

%include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
%include <architecture/msgPayloadDef/PrescribedRotationMsgPayload.h>
