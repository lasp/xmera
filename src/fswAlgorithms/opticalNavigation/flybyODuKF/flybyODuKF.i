// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module flybyODuKF
%{
   #include "flybyODuKF.h"
%}

%include <fswAlgorithms/_GeneralModuleFiles/srukfInterface.i>

%include "flybyODuKF.h"

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/FilterMsgPayload.h>
%include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavUnitVecMsgPayload.h>
