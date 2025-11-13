%module spacecraft
%{
   #include "spacecraft.h"
   #include <simulation/dynamics/_GeneralModuleFiles/hubEffector.h>
%}

%pythoncode %{
from xmera.simulation.gravityEffector import GravBodyVector
%}
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
%include <simulation/dynamics/_GeneralModuleFiles/stateEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynamicObject.h>
%import  <simulation/dynamics/gravityEffector/gravityEffector.i>
%include "spacecraft.h"

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/SCMassPropsMsgPayload.h>

%include <architecture/msgPayloadDef/AttRefMsgPayload.h>

%include <architecture/msgPayloadDef/TransRefMsgPayload.h>


%include <simulation/dynamics/_GeneralModuleFiles/hubEffector.h>
