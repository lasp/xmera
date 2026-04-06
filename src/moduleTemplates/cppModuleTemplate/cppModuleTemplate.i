// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module cppModuleTemplate
%{
   #include "cppModuleTemplate.h"
%}

%include "std_string.i"
%include "swig_conly_data.i"

%import "sys_model.i"
%include "cppModuleTemplate.h"

%include "architecture/msgPayloadDef/ModuleTemplateMsgPayload.h"
