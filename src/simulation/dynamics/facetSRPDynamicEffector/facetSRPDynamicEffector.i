// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module facetSRPDynamicEffector
%{
   #include "facetSRPDynamicEffector.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

// Instantiate templates used by example
%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
%include "facetSRPDynamicEffector.h"

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>
