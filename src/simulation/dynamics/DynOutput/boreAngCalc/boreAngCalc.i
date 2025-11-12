%module boreAngCalc
%{
   #include "boreAngCalc.h"
%}

%include <cmalloc.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "boreAngCalc.h"

%include <architecture/msgPayloadDef/BoreAngleMsgPayload.h>

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
