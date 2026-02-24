// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module lambertValidator
%{
    #include "lambertValidator.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "lambertValidator.h"

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/LambertProblemMsgPayload.h>
%include <architecture/msgPayloadDef/LambertSolutionMsgPayload.h>
%include <architecture/msgPayloadDef/LambertPerformanceMsgPayload.h>
%include <architecture/msgPayloadDef/DvBurnCmdMsgPayload.h>
%include <architecture/msgPayloadDef/LambertValidatorMsgPayload.h>
