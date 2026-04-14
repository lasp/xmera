.. toctree::
   :hidden:

.. _buildExtModules:


Building Custom Modules
=======================

Motivation
----------
It is recommended that third party modules be created within a single folder external to the xmera project tree.
This allows one to replace the xmera project files completely separate from the third party module files.

.. note::

   To learn how to write Xmera module, see the documentation at :ref:`making-modules`.

Usage
-----

CMake's ``add_subdirectory()`` requires that every source directory lives under
the project source tree (``src/``). Because external modules live outside the
repository, you need to create a symbolic link inside ``src/`` that points to
them.

Assuming the external modules folder is called ``externalModules`` and sits next
to the Xmera repository:

.. code-block:: console

    # From the repository root
    ln -s ../../externalModules src/externalModules

Then add the symlinked path to ``XMERA_MODULE_ROOTS`` at configure time:

.. code-block:: console

    cmake --preset base -S src \
        -DXMERA_MODULE_ROOTS="${XMERA_MODULE_ROOTS};${sourceDir}/externalModules/"

Or, more conveniently, create a user preset in ``src/CMakeUserPresets.json``
(see :ref:`CMake Build <cmake-parameters>` for details):

.. code-block:: json

    {
      "version": 2,
      "configurePresets": [
        {
          "name": "with-external",
          "inherits": "base",
          "cacheVariables": {
            "XMERA_MODULE_ROOTS": "${sourceDir}/fswAlgorithms/;${sourceDir}/simulation/;${sourceDir}/moduleTemplates/;${sourceDir}/externalModules/"
          }
        }
      ]
    }

.. code-block:: console

    cmake --preset with-external -S src

Then build and install as usual:

.. code-block:: console

    cmake --build build
    cmake --install build

.. note::

   The symlink only needs to be created once. It is ignored by git and does not
   modify the external modules directory in any way.


Directory Structure
-------------------
The external module inclusion follows a strict directory structure resembling the existing Xmera repository.
A single folder contains all the custom Xmera modules and message definitions in a specific
sub-folder structure shown below.

.. code-block:: none

    custom-modules
    ├── externalModules
    ├── _GeneralModuleFiles
    ├── modules
    │   ├── moduleA
    │   │   ├── tests
    │   │   │   └── test_moduleA.py
    │   │   ├── moduleA.cpp
    │   │   ├── moduleA.h
    │   │   ├── moduleA.i
    │   │   └── moduleA.rst
    │   └── moduleB
    │       ├── tests
    │       │   └── test_moduleB.py
    │       ├── moduleB.cpp
    │       ├── moduleB.h
    │       ├── moduleB.i
    │       └── moduleB.rst
    └── msgPayloadDef
        ├── msgDefinitionA.h
        └── msgDefinitionB.h

- ``custom-modules``:  This is the parent folder that holds all the custom Xmera modules, support files and messages.
    Note that this folder can have any name, `custom-modules` is just a sample name here.

- ``externalModules``: (required, must have this name) This folder contains sub-folders for each custom Xmera
    module. This folder contains the typical source code required to build and test a module. The sub-folders have the
    individual module names.

- ``externalModules/_GeneralModuleFiles``: (optional, but must have this name) This is useful for source code which
    is shared between the multiple modules. If the answer to the question “Will this code need to be included in more
    than one module?” is yes. Then, that support code belongs in ``_GeneralModuleFiles``. While building external
    modules the Xmera build system links ``_GeneralModuleFiles`` to these external modules. The files should be
    located directly inside ``_GeneralModulesFiles``, no sub folders are supported in this module.

- ``msgPayloadDef``: (optional, must have this name) This folder contains the definition of all the message
    header files.


Frequently Asked Questions
--------------------------

#. Can I store my custom Xmera folder on another drive?

- On Linux and macOS, yes.
- On Window this must be on the same drive as the Xmera source

#. How flexible are the folder names?

- While the primary folder, called ``externalModules`` above, can assume any name, the key sub-folders must follow the
  exact naming shown above. If you change this you must write your own ``cmake`` file.

#. How do I import these custom modules when I write a Xmera python simulation script?

- The custom Xmera modules are built into a ``xmera.externalModules`` package.  For example, to load a module
  called ``customCppModule``, use:

.. code-block:: python

        from xmera.externalModules import customCppModule
