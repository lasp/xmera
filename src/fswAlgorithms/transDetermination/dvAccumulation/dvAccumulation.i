%module dvAccumulation
%{
   #include "dvAccumulation.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include "dvAccumulation.h"

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/AccDataMsgPayload.h>
%include <architecture/msgPayloadDef/AccPktDataMsgPayload.h>
