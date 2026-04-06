// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module ExtPulsedTorque
%{
   #include "ExtPulsedTorque.h"
%}

%include <std_string.i>
%include <stdint.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
%include "ExtPulsedTorque.h"
