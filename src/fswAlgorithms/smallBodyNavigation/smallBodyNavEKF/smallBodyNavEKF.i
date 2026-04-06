// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module smallBodyNavEKF
%{
    #include "smallBodyNavEKF.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include "smallBodyNavEKF.h"

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
%include <architecture/msgPayloadDef/SmallBodyNavMsgPayload.h>
%include <architecture/msgPayloadDef/CmdForceBodyMsgPayload.h>
