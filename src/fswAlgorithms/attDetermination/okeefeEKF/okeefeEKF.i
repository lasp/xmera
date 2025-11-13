%module okeefeEKF
%{
   #include "okeefeEKF.h"
   #include <architecture/utilities/ukfUtilities.h>
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "okeefeEKF.h"

%include <architecture/utilities/ukfUtilities.h>

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
%include <architecture/msgPayloadDef/SunlineFilterMsgPayload.h>
%include <architecture/msgPayloadDef/CSSConfigMsgPayload.h>
