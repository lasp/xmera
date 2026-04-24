%module regionsOfInterestPrune
%{
   #include "regionsOfInterestPrune.h"
%}

%include <stdint.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "regionsOfInterestPrune.h"

%include <architecture/msgPayloadDef/FpgaRowColSumMsgPayload.h>
%include <architecture/msgPayloadDef/FpgaThreshImageMsgPayload.h>
%include <architecture/msgPayloadDef/RegionOfInterestMsgPayload.h>
%include <architecture/msgPayloadDef/RegionsIdentifiedMsgPayload.h>
