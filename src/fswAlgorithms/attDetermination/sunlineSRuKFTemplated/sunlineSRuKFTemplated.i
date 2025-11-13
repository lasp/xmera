%module sunlineSRuKFTemplated
%{
   #include "sunlineSRuKFTemplated.hpp"
%}

%include <sys_model.i>
%include <swig_conly_data.i>

%include "sunlineSRuKFTemplated.hpp"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/CSSConfigMsgPayload.h>
%include <architecture/msgPayloadDef/CSSUnitConfigMsgPayload.h>
%include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
%include <architecture/msgPayloadDef/FilterMsgPayload.h>
%include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
