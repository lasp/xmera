%module cssWlsEst
%{
   #include "cssWlsEst.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "cssWlsEst.h"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/CSSConfigMsgPayload.h>
%include <architecture/msgPayloadDef/CSSUnitConfigMsgPayload.h>
%include <architecture/msgPayloadDef/SunlineFilterMsgPayload.h>
%include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
