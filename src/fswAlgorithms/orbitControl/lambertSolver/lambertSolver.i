%module lambertSolver
%{
    #include "lambertSolver.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "lambertSolver.h"

%include <architecture/msgPayloadDef/LambertProblemMsgPayload.h>
%include <architecture/msgPayloadDef/LambertSolutionMsgPayload.h>
%include <architecture/msgPayloadDef/LambertPerformanceMsgPayload.h>
