// SPDX-License-Identifier: ISC
// Copyright (c) 2022, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module spinningBodyTwoDOFStateEffector
%{
   #include "spinningBodyTwoDOFStateEffector.h"
%}

%include <std_string.i>
%include <std_vector.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
%include <simulation/dynamics/_GeneralModuleFiles/stateEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
%include "spinningBodyTwoDOFStateEffector.h"

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/ArrayMotorTorqueMsgPayload.h>

%include <architecture/msgPayloadDef/ArrayEffectorLockMsgPayload.h>

%include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
