// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module chebyPosEphem
%{
   #include "chebyPosEphem.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

STRUCTASLIST(ChebyEphemRecord)
%include "chebyPosEphem.h"

%include <architecture/msgPayloadDef/TDBVehicleClockCorrelationMsgPayload.h>
%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
