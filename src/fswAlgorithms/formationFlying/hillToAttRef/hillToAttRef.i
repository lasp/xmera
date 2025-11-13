%module hillToAttRef
%{
   #include "hillToAttRef.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "hillToAttRef.h"

%include <architecture/msgPayloadDef/HillRelStateMsgPayload.h>
%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
