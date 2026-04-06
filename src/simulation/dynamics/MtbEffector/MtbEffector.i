// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module MtbEffector
%{
    #include "MtbEffector.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
%include "MtbEffector.h"

%include <architecture/msgPayloadDef/MTBCmdMsgPayload.h>

%include <architecture/msgPayloadDef/MagneticFieldMsgPayload.h>

%include <architecture/msgPayloadDef/MTBArrayConfigMsgPayload.h>

%include <architecture/msgPayloadDef/MTBMsgPayload.h>
