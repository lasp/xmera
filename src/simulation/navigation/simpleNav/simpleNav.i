%module simpleNav
%{
   #include "simpleNav.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "simpleNav.h"

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>

%include <architecture/msgPayloadDef/AccDataMsgPayload.h>
