.. _making-new-module:

Making a New C/C++ Xmera Module
==================================

.. note::

   This page provides guidelines on how to get started writing a new Xmera module in C or C++. The intent is to
   offer general tips and best practices. It's recommended to also study existing modules that are similar to the one
   you're creating.

Getting Started
---------------

If this is your first time writing a Xmera module, a good starting point is the ``cModuleTemplate`` located
at ``fswAlgorithms/_cModuleTemplateFolder``. This folder provides a generic ANSI-C Xmera module, but the general
steps are also applicable to C++ modules.

The template includes detailed instructions on:

- Copying an existing Xmera module folder.
- Renaming module methods in ``*.c/cpp``, ``*.h``, ``*.i``, and ``*.rst`` files.
- Properly documenting module functionality and usage.
- Writing and documenting unit tests for the module.

Templates and Examples
----------------------

- For C++ modules, refer to the sample C++ module at :ref:`cppModuleTemplate`, located in
  ``src/simulation/cppModuleTemplate``.

In addition to using these templates, you can base your new module on an existing one with similar functionality,
such as dynamics, sensors, environment models, attitude control, or navigation.

Note: The :ref:`dynamicEffector` and :ref:`stateEffector` classes are special cases. These use a state engine to
manage the numerical integration of differential equations and are handled differently from standard modules.

Base Classes and Custom Methods
-------------------------------

Some modules—like :ref:`magneticFieldCenteredDipole`, :ref:`exponentialAtmosphere`, or :ref:`simpleBattery`—inherit
from base classes. Ensure you understand the expected behaviors defined by these base classes. Override and implement
the relevant ``customXXXX()`` methods to define your module’s specific behavior.

Linear Algebra and Kinematics Support
-------------------------------------

If your module includes vector or tensor math:

- Refer to the :ref:`codingGuidelines` for naming conventions related to matrix representations.
- For C++ modules, Xmera supports the `Eigen library <http://eigen.tuxfamily.org>`_. Keep in mind that Eigen's
  ``.toRotationMatrix()`` method returns the direction cosine matrix (DCM) :math:`[NB]`, not :math:`[BN]`. Xmera's
  Eigen MRP implementation follows this same convention.
- Modules can use the provided ``linearAlgebra.c/h`` for common operations, and ``rigidBodyKinematics.cpp/hpp`` or
  ``rigidBodyKinematics.c/h`` for rigid body kinematics.

Finalizing and Contributing
---------------------------

Once your module is complete and you're ready to contribute it to the Xmera repository, make sure to go through the :ref:`bskModuleCheckoutList` to complete all required review and integration steps.
