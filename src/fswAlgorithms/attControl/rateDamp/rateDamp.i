// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module rateDamp
%{
   #include "rateDamp.h"
%}

%include <std_string.i>

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include "rateDamp.h"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
