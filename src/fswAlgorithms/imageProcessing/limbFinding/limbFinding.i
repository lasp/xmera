%module limbFinding
%{
   #include "limbFinding.h"
%}

%include <stdint.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "limbFinding.h"

%include <architecture/msgPayloadDef/CameraImageMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavLimbMsgPayload.h>
