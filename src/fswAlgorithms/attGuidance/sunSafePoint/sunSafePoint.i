%module sunSafePoint
%{
   #include "sunSafePoint.h"
%}

%include "std_string.i"
%include "swig_conly_data.i"
%include "swig_eigen.i"

%include "sys_model.i"
%include "sunSafePoint.h"

%include "architecture/msgPayloadDef/NavAttMsgPayload.h"
struct NavAttMsg_C;
%include "architecture/msgPayloadDef/AttGuidMsgPayload.h"
