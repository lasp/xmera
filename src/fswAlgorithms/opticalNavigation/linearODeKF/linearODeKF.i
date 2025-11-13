%module linearODeKF
%{
   #include "linearODeKF.h"
%}

%include <fswAlgorithms/_GeneralModuleFiles/ekfInterface.i>

%include "linearODeKF.h"

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/FilterMsgPayload.h>
%include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavUnitVecMsgPayload.h>
