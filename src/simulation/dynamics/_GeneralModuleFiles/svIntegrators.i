// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module svIntegrators

%{
   #include <simulation/dynamics/_GeneralModuleFiles/stateVecIntegrator.h>
   #include <simulation/dynamics/_GeneralModuleFiles/Rk4Integrator.h>
   #include <architecture/_GeneralModuleFiles/sys_model.h>
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/stateVecIntegrator.h>
%include <simulation/dynamics/_GeneralModuleFiles/Rk4Integrator.h>
