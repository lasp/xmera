// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module vscmgStateEffector
%{
   #include "vscmgStateEffector.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
%include <simulation/dynamics/_GeneralModuleFiles/stateEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynamicObject.h>
%include "vscmgStateEffector.h"

%include <architecture/msgPayloadDef/VSCMGCmdMsgPayload.h>

%include <architecture/msgPayloadDef/VSCMGSpeedMsgPayload.h>

%include <architecture/msgPayloadDef/VSCMGArrayTorqueMsgPayload.h>

%include <architecture/msgPayloadDef/VSCMGConfigMsgPayload.h>

%include <architecture/utilities/macroDefinitions.h>

%include <std_vector.i>
