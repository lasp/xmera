%module planetHeading
%{
   #include "planetHeading.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "planetHeading.h"
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/BodyHeadingMsgPayload.h>
