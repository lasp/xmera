Executive Summary
-----------------

The SPICE interface class gets time and planetary or spacecraft body information from the JPL ephemeris library


The module
:download:`PDF Description </../../src/simulation/environment/spiceInterface/_Documentation/Xmera-SPICE_INTERFACE20170712.pdf>`
contains further information on this module's function,
how to run it, as well as testing.



Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - spiceTimeOutMsg
      - :ref:`SpiceTimeMsgPayload`
      - spice time sampling output message
    * - epochInMsg
      - :ref:`EpochMsgPayload`
      - (optional) input epoch message
    * - planetStateOutMsgs
      - :ref:`SpicePlanetStateMsgPayload`
      - vector of planet state output messages
    * - secondaryStateOutMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - (optional) state output message for a secondary body attached to one of the
        planets in ``planetStateOutMsgs``
    * - scStateOutMsgs
      - :ref:`SCStatesMsgPayload`
      - vector of spacecraft state messages
    * - attRefStateOutMsgs
      - :ref:`AttRefMsgPayload`
      - vector of spacecraft attitude reference state output messages
    * - transRefStateOutMsgs
      - :ref:`TransRefMsgPayload`
      - vector of spacecraft translation reference state output messages


User Guide
----------
This module uses the JPL Spice software to determine the position and orientation of both a celestial body, or a spacecraft.
The appropriate Spice kernel must be loaded up to provide the state information for the selected body names.

To setup a celestial body, use the module method ``addPlanetNames(vector<string>)`` in python.  Multiple object names can be
provided by providing a list of names.  With each module update cycle, the corresponding celestial body
states are provided in the vector of output messages ``planetStateOutMsgs[]``.  Note that the vector elements are in
the order that the celestial body names were added.

To use this module to read in spacecraft states from a spice kernel, then the spacecraft Spice name is added using
the method ``addSpacecraftNames(vector<string>)``.  The module provides a vector out corresponding spacecraft state output
messages in three different formats.

- ``scStateOutMsgs[]``: these are the :ref:`SCStatesMsgPayload` state output messages that the :ref:`spacecraft` module provides
- ``attRefStateOutMsgs[]``: these are the attitude reference messages :ref:`AttRefMsgPayload`.  These are useful to
  only prescribe the spacecraft attitude motion.
- ``transRefStateOutMsgs[]``: these are the translational reference message :ref:`TransRefMsgPayload`.  These are useful to only
  prescribe the translational motion and leave the attitude motion free.

Secondary Body
^^^^^^^^^^^^^^
A ``SecondaryBody`` can be attached to one of the celestial bodies in ``planetStateOutMsgs`` to model
a small companion (for example, the secondary of a binary asteroid).  Each module update writes a
:ref:`SpicePlanetStateMsgPayload` on ``secondaryStateOutMsg`` whose ``PositionVector`` is the primary's
SPICE-derived position plus a configurable offset, and whose ``PlanetName`` is set to the secondary's
name (the primary's planet message is not modified).

Configure a secondary body in python::

    secondary = spiceInterface.SecondaryBody()
    secondary.secondaryName = "companion"
    secondary.positionOffset = [1000.0, 2000.0, -3000.0]  # m, in the inertial frame
    secondary.orbitalPeriod = 11.92 * 3600.0              # s; 0 (default) keeps the offset static
    spice.setOffsetBody("primaryName", secondary)

When a non-zero ``orbitalPeriod`` ``T`` is set, the secondary moves on a circle of radius
:math:`|\boldsymbol{r}_{\text{offset}}|` centered on the primary, at angular rate
:math:`\omega = 2\pi/T`.  The orbit plane has normal
:math:`\boldsymbol{r}_{\text{offset}} \times (\boldsymbol{r}_{\text{offset}} \times \hat{Z})`,
which keeps the plane tilted toward the equator (perpendicular to ``Z``) rather than running
through the poles.  The secondary starts at ``positionOffset`` at :math:`t = 0` and a quarter
period later passes through a vector that lies in the equatorial (``XY``) plane.  When the
offset is parallel to ``Z`` the construction degenerates and the orbit plane normal is
instead :math:`\boldsymbol{r}_{\text{offset}} \times (\boldsymbol{r}_{\text{offset}} \times \hat{X})`.
With ``orbitalPeriod = 0`` the offset is held constant.

To stop publishing ``secondaryStateOutMsg`` without discarding the configuration, call
``spice.passivateSecondary()``.  The stored ``SecondaryBody`` is preserved, and a subsequent
``setOffsetBody()`` call re-activates the output.

Limitations
"""""""""""
The secondary body model is a simple kinematic prescription, not a dynamics integration.
Specifically:

- **Circular orbit only.**  The offset is rotated at a constant angular rate
  :math:`2\pi/T` about the primary.  There is no eccentricity, perturbation, or
  two-body integration; supplying an ``orbitalPeriod`` does not solve any
  equations of motion.
- **Orbit plane derived from the offset and J2000 Z.**  The plane normal is
  :math:`\boldsymbol{r}_{\text{offset}} \times (\boldsymbol{r}_{\text{offset}} \times \hat{Z})`
  (or substitute :math:`\hat{X}` for :math:`\hat{Z}` when offset is parallel to
  ``Z``); arbitrary user-specified orbital planes are not supported.
- **Single secondary per ``SpiceInterface`` instance.**  Only one body in
  ``planetStateOutMsgs`` can have a secondary attached at a time.  A second
  binary system requires a second ``SpiceInterface`` instance.
- **Velocity is inherited, not orbital.**  ``secondaryStateOutMsg.VelocityVector``
  is copied from the primary; it does not include the secondary's
  :math:`\boldsymbol{\omega}\times\boldsymbol{r}` contribution.  Consumers that
  need a kinematically consistent velocity must compute it themselves.
- **Initial phase set by the offset vector.**  At :math:`t = 0` the secondary is
  located at the configured ``positionOffset``.  To start at a different phase,
  rotate the offset vector before calling ``setPositionOffset``.
- **Attitude (J20002Pfix) inherited from the primary.**  No body-frame
  orientation of the secondary is computed.
