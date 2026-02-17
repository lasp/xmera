// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module stepperMotorController
%{
   #include "stepperMotorController.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>

%include "stepperMotorController.h"
%include "stepperMotorControllerAlgorithm.h"

%include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
%include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>
