%module flybyODuKF
%{
   #include "flybyODuKF.h"
%}

%include <fswAlgorithms/_GeneralModuleFiles/srukfInterface.i>

%include "flybyODuKF.h"

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/FilterMsgPayload.h>
%include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavUnitVecMsgPayload.h>
