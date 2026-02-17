// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module simplePowerMonitor
%{
    #include "simplePowerMonitor.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/power/_GeneralModuleFiles/powerStorageBase.h>
%include "simplePowerMonitor.h"
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/msgPayloadDef/PowerNodeUsageMsgPayload.h>

%include <architecture/msgPayloadDef/PowerStorageStatusMsgPayload.h>
