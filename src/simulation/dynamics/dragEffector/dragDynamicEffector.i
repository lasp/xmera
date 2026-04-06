// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module dragDynamicEffector
%{
   #include "dragDynamicEffector.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

// Instantiate templates used by example
%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>

%include "dragDynamicEffector.h"

%include <architecture/msgPayloadDef/AtmoPropsMsgPayload.h>
