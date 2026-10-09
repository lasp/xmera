// SPDX-License-Identifier: ISC

%module sunlineSRuKFTemplated
%{
   #include "sunlineSRuKFTemplated.hpp"
%}

%include <sys_model.i>
%include <swig_conly_data.i>

%include "sunlineSRuKFTemplated.hpp"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%import <architecture/msgPayloadDef/CSSConfigMsgPayload.h>
%import <architecture/msgPayloadDef/CSSUnitConfigMsgPayload.h>
%include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
%include <architecture/msgPayloadDef/FilterMsgPayload.h>
%include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
