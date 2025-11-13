%module rateMsgConverter
%{
   #include "rateMsgConverter.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "rateMsgConverter.h"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/IMUSensorBodyMsgPayload.h>
