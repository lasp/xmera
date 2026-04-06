// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module sepPoint
%{
   #include "sepPoint.h"
%}

%include <std_string.i>

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include "sepPoint.h"
%include <fswAlgorithms/attGuidance/_GeneralModuleFiles/constrainedAxisPointingLibrary.h>
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/BodyHeadingMsgPayload.h>
%include <architecture/msgPayloadDef/InertialHeadingMsgPayload.h>
%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
