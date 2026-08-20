.. _messaging-mission-parameters:

Configuring Mission Parameters
==============================

Several message payloads and modules size their arrays from a small set of
project-wide constants: sensor counts, effector counts, etc. These constants
live in a single header:

``mission/parameters.h``

The default values in Xmera are generously sized so that a range of spacecraft
configurations are possible without recompilation. A mission that needs
different limits or wants to take advantage of smaller sizes array sizes can
supply its own values without editing the Xmera defaults.

Mission Constants
-----------------

============================ ======= =====================================
Constant                     Default Meaning
============================ ======= =====================================
``MAX_KEY_POINTS``           5000    Optical flow key points
``MAX_NUM_CSS_SENSORS``      32      Coarse sun sensors in a constellation
``MAX_EFF_CNT``              36      Generic effectors (thrusters, etc.)
``RW_EFF_CNT``               36      Reaction wheels
``MAX_N_CSS_MEAS``           32      Coarse sun sensor measurements
``MAX_SICP_POINTS``          5000    Point-cloud points
``SICP_POINT_DIM``           3       Point-cloud point dimension
``MAX_SICP_ITERATIONS``      250     Point-cloud registration iterations
``MAX_NUMBER_REGIONS``       3       Regions of interest
============================ ======= =====================================

How the Override Works
----------------------

Payloads and modules include ``<mission/parameters.h>``, which resolves to
whichever ``mission/parameters.h`` comes first on the include path.

The ``XMERA_MISSION_PARAMETERS_DIR`` CMake cache variable decides in which
directory the parameters.h is found. It defaults to ``src/defaults``, so
an unconfigured build picks up ``src/defaults/mission/parameters.h`` and
the values in the table above. Pointing the variable somewhere else swaps
the header out.

Supplying Your Own Values
-------------------------

Create a header at ``mission/parameters.h`` inside a directory of your choice,
defining each constant:

.. code-block:: c

    #ifndef MISSION_PARAMETERS_H
    #define MISSION_PARAMETERS_H

    // clang-format off
    #define MAX_KEY_POINTS 5000
    #define MAX_NUM_CSS_SENSORS 8
    #define MAX_EFF_CNT 12
    #define RW_EFF_CNT 4
    #define MAX_N_CSS_MEAS 8
    #define MAX_SICP_POINTS 5000
    #define SICP_POINT_DIM 3
    #define MAX_SICP_ITERATIONS 250
    #define MAX_NUMBER_REGIONS 3
    // clang-format on

    #endif

Point the build at the directory that *contains* the ``mission`` folder:

.. code-block:: bash

    cmake --preset base -DXMERA_MISSION_PARAMETERS_DIR=/path/to/myMission

With the example layout above the file is at
``/path/to/myMission/mission/parameters.h``. Configuring prints the header the
build resolved, which is worth checking when an override appears to have no
effect:

.. code-block:: text

    -- Using mission parameters from /path/to/myMission/mission/parameters.h

A replacement header applies completely, so it must define every constant in
the table above. Omitting one is not reported while configuring; it results
in a compilation error at the point where the constant is used.

Write the values as plain integers, inside the ``clang-format off`` guard shown
above. SWIG also reads this header, to make the constants available to the
Python layer, and its preprocessor reads the quote in a digit separator such as
``5'000`` as the start of a character literal. It then discards the constants
that follow, which surfaces much later as a missing attribute on
``xmera.architecture.messaging`` rather than as a build error. The guard is
needed because this project's ``.clang-format`` sets ``IntegerLiteralSeparator``,
which would otherwise insert those separators for you.
