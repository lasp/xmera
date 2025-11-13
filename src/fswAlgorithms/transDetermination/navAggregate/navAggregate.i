%module navAggregate
%{
   #include "navAggregate.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

STRUCTASLIST(AggregateAttInput)
STRUCTASLIST(AggregateTransInput)
%include "navAggregate.h"
%include "navAggregateAlgorithm.h"
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
