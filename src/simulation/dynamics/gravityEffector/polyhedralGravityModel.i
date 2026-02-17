// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module(package="xmera.simulation") polyhedralGravityModel
%{
   #include "polyhedralGravityModel.h"
   #include <memory>
%}

%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%import "gravityModel.i"

%include <std_shared_ptr.i>
%shared_ptr(PolyhedralGravityModel)

%include "polyhedralGravityModel.h"

%extend PolyhedralGravityModel {
   %pythoncode %{
      def loadFromFile(self, fileName: str):
          """Loads the vertices and facet data from the given file."""
          from xmera.simulation.gravityEffector import loadPolyFromFile
          loadPolyFromFile(fileName, self)
          return self
   %}
}
