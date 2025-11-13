%module exponentialAtmosphere
%{
    #include "exponentialAtmosphere.h"
%}

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>
%include <std_string.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/environment/_GeneralModuleFiles/atmosphereBase.h>
%include "exponentialAtmosphere.h"

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/AtmoPropsMsgPayload.h>
