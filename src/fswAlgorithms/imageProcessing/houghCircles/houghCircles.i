%module houghCircles
%{
   #include "houghCircles.h"
%}

%include <stdint.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "houghCircles.h"

%include <architecture/msgPayloadDef/OpNavCirclesMsgPayload.h>
%include <architecture/msgPayloadDef/CameraImageMsgPayload.h>
