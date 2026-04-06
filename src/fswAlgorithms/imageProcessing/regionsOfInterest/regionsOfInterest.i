%module regionsOfInterest
%{
   #include "regionsOfInterest.h"
%}

%include <stdint.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "regionsOfInterest.h"

%include <architecture/msgPayloadDef/RegionsIdentifiedMsgPayload.h>
%include <architecture/msgPayloadDef/RegionOfInterestMsgPayload.h>
