// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module ephemDifferenceWithUncertainty
%{
   #include "ephemDifferenceWithUncertainty.h"
%}

%include <stdint.i>
%include <std_string.i>
%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "ephemDifferenceWithUncertainty.h"

%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
%include <architecture/msgPayloadDef/NavTransMsgPayload.h>

%include <architecture/msgPayloadDef/FilterMsgPayload.h>
