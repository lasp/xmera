.. _cmake-parameters:

.. toctree::
   :hidden:

CMake Build
-----------

Xmera uses `CMake presets <https://cmake.org/cmake/help/latest/manual/cmake-presets.7.html>`__
to simplify configuration. The presets are defined in ``src/CMakePresets.json``.

Prerequisites
~~~~~~~~~~~~~

- CMake 3.27+
- `Ninja <https://ninja-build.org/>`__ build system
- `vcpkg <https://vcpkg.io/>`__ with the ``VCPKG_ROOT`` environment variable set
- Python 3 (with development headers)
- SWIG

Presets
~~~~~~~

.. list-table:: Available Configure Presets
    :widths: 20 80
    :header-rows: 1

    * - Preset
      - Description
    * - ``base``
      - Default development build (Debug, Ninja). Builds all module groups.
    * - ``ci-test``
      - Inherits ``base`` with ``CMAKE_BUILD_TYPE=Release``.
    * - ``fuzz-smoke-test``
      - Inherits ``ci-test`` with fuzz tests enabled.
    * - ``fuzz-test``
      - Inherits ``fuzz-smoke-test`` with full fuzzing mode.

Cache Variables
~~~~~~~~~~~~~~~

The presets set sensible defaults for the following variables. You can override
them with ``-D`` on the command line.

.. list-table:: CMake Cache Variables
    :widths: 30 15 55
    :header-rows: 1

    * - Variable
      - Default
      - Description
    * - ``CMAKE_BUILD_TYPE``
      - ``Debug``
      - Build type (``Debug``, ``Release``, ``RelWithDebInfo``)
    * - ``CMAKE_INSTALL_PREFIX``
      - ``<repo>/dist``
      - Where ``cmake --install`` places the Python package
    * - ``XMERA_MODULE_ROOTS``
      - (all groups)
      - Semicolon-separated list of source directories containing modules
    * - ``XMERA_ENABLE_GROUPS``
      - ``simulation;fswAlgorithms;moduleTemplates``
      - Semicolon-separated list of module group prefixes to build
    * - ``XMERA_ENABLE_INTERNAL``
      - ``YES``
      - Build internal/private modules
    * - ``XMERA_ENABLE_FUZZTESTS``
      - ``OFF``
      - Fetch and build fuzz targets with Google FuzzTest
    * - ``URL_SPICE_KERNEL``
      - (JPL URL)
      - URL from which to download the ``de430.bsp`` SPICE kernel

Custom User Presets
~~~~~~~~~~~~~~~~~~~

You can create a ``src/CMakeUserPresets.json`` file to define your own presets
without modifying the project-tracked ``CMakePresets.json``. User presets
inherit from project presets and are ignored by git (the file is
``.gitignore``-d by convention).

The file must declare ``"version": 2`` (or higher) and a
``"configurePresets"`` array. Each preset can use ``"inherits"`` to pull in
defaults from a project preset and then override only what it needs.

**Example — Release build:**

.. code-block:: json

    {
      "version": 2,
      "configurePresets": [
        {
          "name": "release",
          "inherits": "base",
          "cacheVariables": {
            "CMAKE_BUILD_TYPE": "Release"
          }
        }
      ]
    }

.. code-block:: console

    cmake --preset release -S src
    cmake --build build

**Example — Build only simulation modules:**

.. code-block:: json

    {
      "version": 2,
      "configurePresets": [
        {
          "name": "sim-only",
          "inherits": "base",
          "cacheVariables": {
            "XMERA_ENABLE_GROUPS": "simulation"
          }
        }
      ]
    }

**Example — Include external modules:**

CMake requires all source directories to live under the project source tree.
Symlink your external modules folder into ``src/`` first (see
:ref:`buildExtModules` for the full workflow), then reference the symlink:

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

You can list all available presets (project + user) with:

.. code-block:: console

    cmake --list-presets -S src

Configure, Build, Install
~~~~~~~~~~~~~~~~~~~~~~~~~~

All commands are run from the repository root.

1. **Configure** using a preset (runs vcpkg and generates build files):

   .. code-block:: console

       cmake --preset base -S src

   To override a variable, append ``-D``:

   .. code-block:: console

       cmake --preset base -S src -DCMAKE_BUILD_TYPE=Release

2. **Build** (compiles C++ libraries and generates Python bindings):

   .. code-block:: console

       cmake --build build

   To build a single target:

   .. code-block:: console

       cmake --build build --target <target-name>

3. **Install** (copies the Python package into ``dist/``):

   .. code-block:: console

       cmake --install build

After installation the ``dist/xmera`` directory contains the importable Python
package.

.. note::

   If you use ``CMAKE_INSTALL_MODE=REL_SYMLINK_OR_COPY`` in your environment,
   **unset it** before configure/build steps — it can break vcpkg symlinks.
   Only set it for the final ``cmake --install`` step if you want symlinks for
   Xmera's own files.
