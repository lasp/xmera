%module relativeODuKF
%{
   #include "relativeODuKF.h"
   #include <architecture/utilities/ukfUtilities.h>
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "relativeODuKF.h"

%include <architecture/utilities/ukfUtilities.h>

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavFilterMsgPayload.h>
