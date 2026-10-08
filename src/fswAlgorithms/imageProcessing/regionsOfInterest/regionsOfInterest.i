%module regionsOfInterest
%{
   #include "regionsOfInterest.h"
%}

%include <stdint.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <attribute.i>
%attribute(RegionsOfInterest, int32_t, maxRoiSeparation, getMaxRoiSeparation, setMaxRoiSeparation)
%attribute(RegionsOfInterest, Eigen::Vector2i, windowCenter, getWindowCenter, setWindowCenter)

%include "regionsOfInterest.h"

%include <architecture/msgPayloadDef/RegionsIdentifiedMsgPayload.h>
%include <architecture/msgPayloadDef/RegionOfInterestMsgPayload.h>
