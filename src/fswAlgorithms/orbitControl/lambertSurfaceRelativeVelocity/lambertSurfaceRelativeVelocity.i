// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module lambertSurfaceRelativeVelocity
%{
    #include "lambertSurfaceRelativeVelocity.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include "lambertSurfaceRelativeVelocity.h"

%include <architecture/msgPayloadDef/LambertProblemMsgPayload.h>
%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
%include <architecture/msgPayloadDef/DesiredVelocityMsgPayload.h>
