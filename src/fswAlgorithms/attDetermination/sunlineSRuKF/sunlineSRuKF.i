%module sunlineSRuKF
%{
   #include "sunlineSRuKF.h"
%}

%include <fswAlgorithms/_GeneralModuleFiles/srukfInterface.i>

%include "sunlineSRuKF.h"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/CSSConfigMsgPayload.h>
%include <architecture/msgPayloadDef/CSSUnitConfigMsgPayload.h>
%include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
%include <architecture/msgPayloadDef/FilterMsgPayload.h>
%include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
