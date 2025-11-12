%module groundMapping
%{
    #include "groundMapping.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <std_vector.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "groundMapping.h"

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/AccessMsgPayload.h>

%include <architecture/msgPayloadDef/GroundStateMsgPayload.h>
