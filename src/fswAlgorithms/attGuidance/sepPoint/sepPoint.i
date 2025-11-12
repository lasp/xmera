%module sepPoint
%{
   #include "sepPoint.h"
%}

%include <std_string.i>

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "sepPoint.h"
%include <fswAlgorithms/attGuidance/_GeneralModuleFiles/constrainedAxisPointingLibrary.h>
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/BodyHeadingMsgPayload.h>
%include <architecture/msgPayloadDef/InertialHeadingMsgPayload.h>
%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
