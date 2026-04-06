// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module reactionWheelStateEffector
%{
   #include "reactionWheelStateEffector.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
%include <simulation/dynamics/_GeneralModuleFiles/stateEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynamicObject.h>
%include <simulation/dynamics/reactionWheels/reactionWheelSupport.h>
%include "reactionWheelStateEffector.h"
%include <architecture/utilities/macroDefinitions.h>

%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>

%include <architecture/msgPayloadDef/RWCmdMsgPayload.h>

%include <architecture/msgPayloadDef/RWConfigMsgPayload.h>
%include <architecture/msgPayloadDef/RWConfigLogMsgPayload.h>

%include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>


%include <std_vector.i>
namespace std {
    %template(RWConfigPointerVector) vector<RWConfigMsgPayload *, allocator<RWConfigMsgPayload *> >;
}
