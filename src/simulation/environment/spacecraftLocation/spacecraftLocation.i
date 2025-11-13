%module spacecraftLocation
%{
    #include "spacecraftLocation.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "spacecraftLocation.h"

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/AccessMsgPayload.h>
