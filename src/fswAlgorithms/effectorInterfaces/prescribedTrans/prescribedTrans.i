// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module prescribedTrans
%{
   #include "prescribedTrans.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "prescribedTrans.h"

%include <architecture/msgPayloadDef/PrescribedTranslationMsgPayload.h>

%include <architecture/msgPayloadDef/LinearTranslationRigidBodyMsgPayload.h>
