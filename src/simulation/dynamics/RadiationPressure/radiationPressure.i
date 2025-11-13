%module radiationPressure
%{
   #include "radiationPressure.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "radiationPressure.h"

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/EclipseMsgPayload.h>


%pythoncode "parseSRPLookup.py"
