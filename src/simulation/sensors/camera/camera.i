%module camera
%{
   #include "camera.h"
%}

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <stdint.i>
%include <std_string.i>
%include <std_vector.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "camera.h"

%include <architecture/msgPayloadDef/CameraModelMsgPayload.h>
%include <architecture/msgPayloadDef/CameraImageMsgPayload.h>

%include <architecture/msgPayloadDef/CameraConfigMsgPayload.h>
