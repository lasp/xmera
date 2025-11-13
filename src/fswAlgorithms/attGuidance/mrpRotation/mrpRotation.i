%module mrpRotation
%{
   #include "mrpRotation.h"
%}

%include <attribute.i>
%attribute(MrpRotation, Eigen::Vector3d, sigma_RR0, getSigmaRR0, setSigmaRR0)
%attribute(MrpRotation, Eigen::Vector3d, omega_RR0_R, getOmegaRR0, setOmegaRR0)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "mrpRotation.h"
%include "mrpRotationAlgorithm.h"

%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
%include <architecture/msgPayloadDef/AttStateMsgPayload.h>
