%module eclipse
%{
   #include "eclipse.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "eclipse.h"
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>

%include <architecture/msgPayloadDef/EclipseMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>
