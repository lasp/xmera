%module celestialTwoBodyPoint
%{
   #include "celestialTwoBodyPoint.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "celestialTwoBodyPointAlgorithm.h"
%include "celestialTwoBodyPoint.h"

%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
