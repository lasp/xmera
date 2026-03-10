.. _makingModules-5:

Support Files
=============

The folder ``src/architecture/utilities`` contains a number of C code support libraries that simplify the mathematics of writing Xmera modules.  This page highlights some common libraries used.


`astroConstants.h`
    Contains a range of orbital mechanics related variable definition.


`eigenMRP.h`
    Provides MRP capabilities to the `Eigen library <https://eigen.tuxfamily.org/>`__.

`eigenSupport.h`
    Provides a range of helper function to convert between C array variables and `Eigen library <https://eigen.tuxfamily.org/>`__ vectors and matrices.

`discretize.h`
    Provides functions to discretize a real number.

`gauss_markov.h`
    Provides functions to apply a second-order bounded Gauss-Markov random walk on top of an upper level process.

`geodeticConversion.h`
    Provides a collection of functions to convert to and from planet centered frames.

`keplerianOrbit.h`
    Class that represents an elliptical orbit and provides a coherent set of common outputs such as position and velocity, orbital period, semi-parameter, etc. It uses the utility orbitalMotion to do orbital element to position and velocity conversion.

`linearAlgebra.h`
    Provides a collection of functions to perform 2D, 3D, 4D and N-dimensional matrix math in C.

`macroDefinitions.h`
    Provides a collection of convenient macro definitions.

`orbitalMotion.h`
    Provides a collection of orbital mechanics related functions.

`rigidBodyKinematics.h`
    Provides a collection of rigid body kinematics transformations.  This includes functions to map between a range of attitude coordinates.

`saturate.h`
    Used to saturate an output variable.

`signalCondition.h`
    Provides a low-pass filter to an output variable.

`simDefinitions.h`
    Provides common simulation related definitions such as default epoch states, etc.
