// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module boreAngCalc
%{
   #include "boreAngCalc.h"
%}

%include <cmalloc.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include "boreAngCalc.h"

%include <architecture/msgPayloadDef/BoreAngleMsgPayload.h>

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
