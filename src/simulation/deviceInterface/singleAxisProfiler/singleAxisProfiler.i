// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module singleAxisProfiler
%{
   #include "singleAxisProfiler.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include "singleAxisProfiler.h"

%include <architecture/msgPayloadDef/StepperMotorMsgPayload.h>
%include <architecture/msgPayloadDef/PrescribedRotationMsgPayload.h>
