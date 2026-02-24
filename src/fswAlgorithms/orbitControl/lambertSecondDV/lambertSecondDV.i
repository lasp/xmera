// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module lambertSecondDV
%{
    #include "lambertSecondDV.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "lambertSecondDV.h"

%include <architecture/msgPayloadDef/LambertSolutionMsgPayload.h>
%include <architecture/msgPayloadDef/DesiredVelocityMsgPayload.h>
%include <architecture/msgPayloadDef/DvBurnCmdMsgPayload.h>
