// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module simplePowerSink
%{
    #include "simplePowerSink.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/power/_GeneralModuleFiles/powerNodeBase.h>
%include "simplePowerSink.h"
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/msgPayloadDef/PowerNodeUsageMsgPayload.h>

%include <architecture/msgPayloadDef/DeviceStatusMsgPayload.h>
