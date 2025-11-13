%module stateArchitecture
%{
   #include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
   #include <architecture/utilities/eigenSupport.h>
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
%include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
%include <architecture/utilities/eigenSupport.h>
