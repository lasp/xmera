// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module ephemeridesRecenter
%{
   #include "ephemeridesRecenter.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>
%include <std_string.i>
%include <std_array.i>
%template(StringArray10) std::array<std::string, MAX_NUM_CHANGE_BODIES>;

%include "ephemeridesRecenter.h"
%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
