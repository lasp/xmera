.. toctree::
   :hidden:


.. _installLinux:

Setup On Linux
==============

Development Environment
-----------------------

.. caution::

    Building Xmera on Linux using Intel processors is being regularly tested. With Xmera version 2.1.5 we added
    experimental support for Linux on ARM processors.  However, this option is not regularly tested.

.. Note::

    Depending on your system setup, administrative permissions (sudo or su) may be required to install these
    dependencies. Some distributions of Linux will use other package management commands such as ``yum``, ``dnf``, of
    ``pgk``.

On a new Linux (assuming Ubuntu 24.04) system various developer packages and support libraries are required

.. code-block:: console

   apt-get update
   apt-get install git
   apt-get install build-essential
   apt-get install pkgconf
   apt-get install python3
   apt-get install python3-dev
   apt-get install python3-pip
   apt-get install python3-setuptools
   apt-get install python3-tk
   apt-get install swig
   python3 --version

Python virtual environment with same version as Python. For example, python3.9-venv for python3.9.x

.. code-block:: console

    apt-get install python3.x-venv

A C/C++ Compiler: This is included by default with most Linux systems (``gcc``), but is necessary to build Xmera.

Xmera uses `vcpkg <https://vcpkg.io/en/>`_ for managing build dependencies

Carry out steps 1-3 detailed in the `vcpkg documentation
<https://learn.microsoft.com/en-us/vcpkg/get_started/get-started?pivots=shell-bash>`_

Setting up Python
-----------------

Using a Python Virtual Environment

.. attention::

    We strongly recommend using a python virtual environment while installing xmera or running
    xmera modules. For more info, `read this <https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/>`__.
    The virtual environment facilitates installing packages specific to this environment such that they won't interfere
    with other python projects.

The following steps show how to create, activate and deactivate a virtual environment. The remaining installation
steps work regardless if done within a virtual environment or not.

Create a virtual environment:

.. code-block:: console

    cd <your-path-to-xmera>/xmera
    python3 -m venv .venv

This creates a hidden folder named `.venv` inside the xmera folder which stores all python packages and environment
information.

Activate the virtual environment when building or running Xmera:

.. code-block:: console

    source .venv/bin/activate

After executing this step you will notice "(.venv)" prepended to your command prompt.

Deactivate the virtual environment to return to the normal command line environment:

.. code-block:: console

    deactivate

CMake: You can install cmake using pip3.  This makes it easy to overcome limitations of which version of ``cmake`` the
``apt-get`` command provides

.. code-block:: console

    pip3 install cmake

Build The Project
-----------------
When all the prerequisite installations are complete, the project can be built as follows

.. code-block:: console

    cd src
    cmake --preset base
    cmake --build ../build --parallel
    CMAKE_INSTALL_MODE=REL_SYMLINK_OR_COPY cmake --install ../build --prefix ../dist
    pip install -e .

The project defines four CMake presets;

- base - Debug build profile
- ci-test - Inherits from "base" with a Release build profile. Simulation execution is faster with a Release build
  profile.
- fuzz-smoke-test - Inherits from "ci-test" with fuzz tests enabled
- fuzz-test - Inherits from "fuzz-smoke-test" with fuzzing mode enabled

To view the complete definition of these presets see the presets file in `src/CMakePresets.json`.

CMake presets are predefined configurations stored in JSON files (CMakePresets.json and CMakeUserPresets.json) that
specify build settings such as generators, build directories, and cache variables. Presets streamline project setup by
allowing consistent, shareable, and repeatable builds across teams and environments without requiring complex
command-line arguments. One can configure additional project :ref:`CMake build parameters <cmake-parameters>` in a
custom CMakeUserPresets.json.

To clean a build

.. code-block:: console

    cmake --build ../build --target clean

To clean and then build

.. code-block:: console

    cmake --build ../build --clean-first

To test your setup you can run one of the :ref:`examples`

.. code-block:: console

    python3 xmera/examples/scenarioBasicOrbit.py
