// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module stepperMotor
%{
   #include "stepperMotor.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include "stepperMotor.h"

%include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>

%include <architecture/msgPayloadDef/StepperMotorMsgPayload.h>
