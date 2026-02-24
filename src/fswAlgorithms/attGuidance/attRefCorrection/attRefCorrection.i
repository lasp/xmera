// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module attRefCorrection
%{
    #include "attRefCorrection.h"
%}

%include <attribute.i>
%attribute(AttRefCorrection, Eigen::Vector3d, sigma_RR0, getSigmaRR0, setSigmaRR0)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "attRefCorrection.h"

%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
