%module pixelLineConverter
%{
   #include "pixelLineConverter.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
STRUCTASLIST(PixelLineConvertData)

%include "pixelLineConverter.h"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavCirclesMsgPayload.h>
%include <architecture/msgPayloadDef/CameraConfigMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavMsgPayload.h>
