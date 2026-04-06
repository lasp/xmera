// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module prescribedLinearTranslation
%{
   #include "prescribedLinearTranslation.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include "prescribedLinearTranslation.h"

%include <architecture/msgPayloadDef/PrescribedTranslationMsgPayload.h>
%include <architecture/msgPayloadDef/LinearTranslationRigidBodyMsgPayload.h>
