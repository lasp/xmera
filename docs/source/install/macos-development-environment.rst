.. toctree::
   :hidden:


.. _installMacOS:

Setup On macOS
==============

Development Environment
-----------------------
In order to run Xmera on macOS, the following software is necessary

-  From a Terminal window to install the command line tools using,

.. code-block:: console

    xcode-select --install

Install `HomeBrew <http://brew.sh>`__ and then the following tools via HomeBrew

- CMake >= 3.30 < 4.0
- pkg-config >= 0.29.2
- SWIG >= 4.2.1

.. code-block:: console

    brew install cmake
    brew link cmake
    brew install pkg-config
    brew install swig

Xmera uses `vcpkg <https://vcpkg.io/en/>`_ for managing build dependencies

Install vcpkg executable

.. code-block:: console

   brew install vcpkg

Install vcpkg repositories

.. code-block:: console

    git clone https://github.com/microsoft/vcpkg "$HOME/vcpkg"

Set the VCPKG_ROOT environment variable by adding the following line to your interactive shell configuration of choice
e.g. `.bashrc` or `.zshrc`.

.. code-block:: console

    export VCPKG_ROOT="$HOME/vcpkg"
    export PATH=$VCPKG_ROOT:$PATH

Install Python 3
----------------

.. attention::
    If you are running a new Apple computer with the M-series ARM64 processor, be sure to download a
    version of Python that is compatible with M-series processor.  The
    `Python.org <https://python.org>`__ web site contains Universal binaries for Python 3.9 and
    onward.  Regarding the python packages via ``pip`` and ``brew``, the required packages
    can all be installed readily in a native form using the standard installation instructions below.

To install Python 3 on macOS there are two common options

#. (Preferred) Download the installer package from `python.org <https://python.org>`__.  This will configure your
   your macOS environment for Python 3 and can readily be upgraded by downloading a newer installer package.
#. Install python 3 through the `HomeBrew <http://brew.sh>`__ package management system. The site has the
   command line to install homebrew from a terminal window using ``brew install python3``.

Setting up Python
-----------------

.. Note:: The following instructions recommend installing all the required python packages
   either in a virtual environment or in the user ``~/Library/Python`` folder. This has the benefit that
   no ``sudo`` command is required to install and run Xmera, and the user Python folder can readily
   be replaced if needed. If you are familiar with python you can install in other locations as well.

.. Note:: If you wish to use the HomeBrew version of python, or generally have multiple copies of
   python installed on your system, configure the CMake Python paths as described in
   :ref:`customPython` after following these instructions.

.. Note:: We suggest you remove any other python packages (such as Anaconda), or change the path in
   your terminal shell if you really want to keep it.

In the following instructions, be sure to follow the sequence of tasks as outlined below.

Creating and Using A Python Virtual Environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. attention:: We strongly recommend using a python virtual environment while installing xmera or running xmera modules.
    For more info, `read this <https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/>`__.
    The virtual environment facilitates installing packages specific to this environment such that they won't interfere
    with other python projects.

The following steps show how to create, activate ad deactivate a virtual environment.  The remaining installation
steps work regardless if done within a virtual environment or not.

Create a virtual environment

.. code-block:: console

    cd <your-path-to-xmera>/xmera
    python3 -m venv .venv

This creates a hidden folder named `.venv` inside the xmera folder which stores all python packages and environment
information.

Activate the virtual environment when building or running Xmera

.. code-block:: console

    source .venv/bin/activate

After executing this step you will notice "(.venv)" prepended to your command prompt.

Install the project's python dependencies

.. code-block:: console

    pip install -r requirements.txt


Deactivate the virtual environment to return to the normal command line environment

.. code-block:: console

    deactivate

Build The Project
-----------------
When all the prerequisite installations are complete, the project can be built as follows

.. code-block:: console

    cd src
    cmake --preset base
    cmake --build ../dist3
    cmake --install ../dist3
    pip install -e ..

The project defines three CMake presets;

- base - Debug build profile with visualization dependencies
- full - Everything in "base" with OpNav dependencies
- ci-test - Everything in "full" with Release build profile. Simulation execution is faster with a Release build
  profile.

To view the complete definition of these presets see the presets file in `src/CMakePresets.json`.

CMake presets are predefined configurations stored in JSON files (CMakePresets.json and CMakeUserPresets.json) that
specify build settings such as generators, build directories, and cache variables. Presets streamline project setup by
allowing consistent, shareable, and repeatable builds across teams and environments without requiring complex
command-line arguments. One can configure additional project :ref:`CMake build parameters <cmake-parameters>` in a
custom CMakeUserPresets.json.

To clean a build

.. code-block:: console

    cmake --build ../dist3 --target clean

To clean and then build

.. code-block:: console

    cmake --build ../dist3 --clean-first

To test your setup you can run one of the :ref:`examples`

.. code-block:: console

    python3 xmera/examples/scenarioBasicOrbit.py

Mac OS FAQs
-----------

#. Q : Permission denied when using brew

   -  A: Add sudo to the start of the command. If you do not have superuser access, get superuser access.
