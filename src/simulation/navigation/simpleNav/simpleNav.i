// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module simpleNav
%{
   #include "simpleNav.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>

// Expose the fixed-size Gauss-Markov matrices/vectors to Python (get/set as lists or
// numpy arrays, with wrong-size assignments rejected by a ValueError at the boundary).
EIGEN_MAT_WRAP(SimpleNavCovar, 159)
EIGEN_MAT_WRAP(SimpleNavErrorVector, 159)

%include "simpleNav.h"

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>

%include <architecture/msgPayloadDef/AccDataMsgPayload.h>
