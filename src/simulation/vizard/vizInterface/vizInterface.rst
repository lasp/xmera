
Executive Summary
-----------------
This module provides an interface to package up Xmera messages and pass them onto the Vizard
application.  This allows for the Xmera simulation data to be recorded to a file for play-back, or for
live streaming of the simulation data.  It is possible to record the simulation data of a single spacecraft or a
multitude of spacecraft.

Module Assumptions and Limitations
----------------------------------
The module is only able to read Xmera simulation state messages that are accessable from the task group that
is calling ``vizInterface``.

Reset Errors
^^^^^^^^^^^^
The ``reset()`` method examines the module configuration. For each of these conditions, the method stops
with an exception (``std::invalid_argument`` in C++, ``RuntimeError`` in Python):

- The state message of a spacecraft in ``scData`` is not connected.
- A RW, CSS or thruster input message of a spacecraft is not connected.
- For a spacecraft, the size of ``thrInfo`` and the size of ``thrInMsgs`` are not the same.
- The quantity of ``spiceInMsgs`` is more than the quantity of ``gravBodyInformation`` entries.
- A body name in ``gravBodyInformation`` has more than 63 characters.

The exception occurs in ``InitializeSimulation()``, before the first time step.

Image Messages
^^^^^^^^^^^^^^
If the image length from Vizard is more than the quantity of received data, the module writes an image
message with ``valid`` set to 0. That message has an ``imageBufferLength`` of 0 and a null ``imagePointer``.


Message Connection Descriptions
-------------------------------
The following messages are set directly within ``vizInterface``.  Additional messages are set within the
``VizSpacecraftData`` data structures for each spacecraft.


.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Parameter
      - Default
      - Description
    * - opnavImageOutMsgs
      - :ref:`CameraImageMsgPayload`
      - (optional) vector of Image output messages, the corresponding camera configuration input message is setup
        through ``vizInterface.addCamMsgToModule(msg)``
    * - epochInMsg
      - :ref:`EpochMsgPayload`
      - (optional) simulation epoch date/time input msg
    * - spiceInMsgs
      - :ref:`SpicePlanetStateMsgPayload`
      - (optional) vector of input messages of planet Spice data

The ``VizSpacecraftData`` structure, defined in ``vizStructures``, contains a range of input messages for each spacecraft added.

.. list-table:: ``VizSpacecraftData`` input messages per spacecraft
    :widths: 25 25 50
    :header-rows: 1

    * - Parameter
      - Default
      - Description
    * - scStateInMsg
      - :ref:`SCStatesMsgPayload`
      - msg of incoming spacecraft state data
    * - rwInMsgs
      - :ref:`RWConfigLogMsgPayload`
      - (optional) Vector of incoming RW state messages
    * - thrInMsgs
      - :ref:`THROutputMsgPayload`
      - (optional) vector of thruster input messages
    * - cssInMsgs
      - :ref:`CSSConfigLogMsgPayload`
      - (optional) Vector of CSS config log messages


User Guide
----------
The ``vizInterface`` module can be directly configured, or setup using the helper methods in ``vizSupport``.
More information can be found in ``vizardSettings`` page.  The :ref:`scenarioFormationBasic` illustrates and
discusses how to configure ``vizInterface`` for use with multiple satellites.
