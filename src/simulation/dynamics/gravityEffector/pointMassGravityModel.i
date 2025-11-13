%module(package="xmera.simulation") pointMassGravityModel
%{
   #include <simulation/dynamics/_GeneralModuleFiles/pointMassGravityModel.h>
   #include <memory>
%}

%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%import <simulation/dynamics/gravityEffector/gravityModel.i>

%include <std_shared_ptr.i>
%shared_ptr(PointMassGravityModel)

%include <simulation/dynamics/_GeneralModuleFiles/pointMassGravityModel.h>
