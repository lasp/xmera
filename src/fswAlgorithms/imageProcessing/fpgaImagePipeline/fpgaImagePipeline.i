%module fpgaImagePipeline
%{
   #include "fpgaImagePipeline.h"
%}

%include <stdint.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "fpgaImagePipeline.h"

%include <architecture/msgPayloadDef/CameraImageMsgPayload.h>
%include <architecture/msgPayloadDef/FpgaRawImageMsgPayload.h>
%include <architecture/msgPayloadDef/FpgaThreshImageMsgPayload.h>
%include <architecture/msgPayloadDef/FpgaRowColSumMsgPayload.h>
STRUCTASLIST(FpgaRoiEntry)
%include <architecture/msgPayloadDef/FpgaRoiMsgPayload.h>
%include <architecture/msgPayloadDef/FpgaPipelineConfigMsgPayload.h>
