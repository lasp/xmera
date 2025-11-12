%module centerOfBrightness
%{
   #include "centerOfBrightness.h"
%}

%include <stdint.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_array.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "centerOfBrightness.h"

%include <architecture/msgPayloadDef/CameraImageMsgPayload.h>

%include <architecture/msgPayloadDef/OpNavCOBMsgPayload.h>
