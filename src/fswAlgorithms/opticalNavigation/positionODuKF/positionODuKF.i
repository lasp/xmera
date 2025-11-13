%module positionODuKF
%{
   #include "positionODuKF.h"
%}

%include <stdint.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "positionODuKF.h"

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/CameraLocalizationMsgPayload.h>
%include <architecture/msgPayloadDef/FilterMsgPayload.h>
%include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
