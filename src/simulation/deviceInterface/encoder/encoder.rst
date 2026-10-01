Executive Summary
-----------------

The encoder module simulates a reaction wheel speed encoder. It counts the encoder clicks of each wheel in each time step. Then it sends the wheel speed that agrees with that count.
The module also simulates encoder failures with a signal state for each wheel.

Message Connection Descriptions
-------------------------------
The table that follows shows the input and output messages of the module. The user connects the messages in Python.
The message type column links to the message structure definition. The description column tells what each message contains.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - rwSpeedInMsg
      - :ref:`RWSpeedMsgPayload`
      - Input reaction wheel speeds and wheel angles.
    * - rwSpeedOutMsg
      - :ref:`RWSpeedMsgPayload`
      - Encoder output wheel speeds and the input wheel angles.

Detailed Module Description
---------------------------

The module simulates two encoder features for each wheel: discretization and signal state. Discretization changes the output wheel speed to a multiple of the encoder resolution.
The signal state simulates a nominal encoder or an encoder failure.

The module operates only on the first ``numRW`` wheels of the input message. The output speeds of the other wheels are zero.

Discretization
~~~~~~~~~~~~~~

The number of clicks per rotation :math:`n` sets the resolution of the encoder. Let :math:`N` be the number of clicks that the encoder counts in one time step:

.. math::
    N = \texttt{trunc}(\Omega_{\text{in}}\Delta t \frac{n}{2\pi} + \Delta N)

Here, :math:`\Omega_{\text{in}}` is the input wheel speed and :math:`\Delta t` is the time step. :math:`\Delta N` is the remaining part of a click from the previous step.
The ``trunc()`` function removes the part of the result after the decimal point. Thus it moves the result toward zero, also for a negative wheel speed.
The module calculates the output wheel speed :math:`\Omega_{\text{out}}` with this equation:

.. math::
    \Omega_{\text{out}} = N\frac{2\pi}{n\Delta t}

Then the module keeps the remaining part of a click for the next step:

.. math::
    \Delta N = \Omega_{\text{in}}\Delta t \frac{n}{2\pi} + \Delta N - N

The value of :math:`\Delta N` is always more than -1 and less than 1. Thus the difference between the output wheel angle and the input wheel angle is always less than one click.
A small value of :math:`n` causes large discretization errors. The errors are also large when the wheel speed is near zero.

Zero Time Step
~~~~~~~~~~~~~~

The module cannot count clicks on a step with a zero time step. This occurs on the first step after reset.
On such a step, a nominal encoder sends the input wheel speed without discretization. The module uses the off and stuck signal states as on all other steps.

Signal State
~~~~~~~~~~~~

Each wheel has a signal state of the type ``EncoderSignal``:

- ``Nominal``: The encoder operates correctly. The module uses the discretization above.
- ``Off``: The output wheel speed is zero. The module also sets the remaining part of a click to zero. This simulates an encoder that is off.
- ``Stuck``: The output wheel speed and the remaining part of a click do not change from the previous step.

All wheels start in the ``Nominal`` state. Reset does not change the signal states. Thus the user can set a signal state before the simulation starts.

Wheel Angles
~~~~~~~~~~~~

The encoder does not measure the wheel angles. The module sends the input ``wheelThetas`` unchanged on each step.


Model Functions
---------------

The functions of the encoder model are:

- **Discretization:** The module changes each wheel speed to a multiple of the encoder resolution.
- **Signal State:** The module changes the output wheel speed of each wheel to agree with its signal state.


Model Assumptions and Limitations
---------------------------------

The module uses these assumptions:

- **The wheel speed is constant during each time step:** The module uses one Euler integration step to calculate the number of clicks.
- **All encoders have the same resolution:** The module uses one value of clicks per rotation for all reaction wheels.


User Guide
----------

This section shows examples of the module setup.

Module Setup
~~~~~~~~~~~~

The constructor has two necessary parameters: the number of reaction wheels and the number of clicks per rotation.
The number of reaction wheels must be from 1 to ``RW_EFF_CNT``. The number of clicks per rotation must be more than zero.

.. code-block:: python
    :linenos:

    wheelSpeedEncoder = encoder.Encoder(numRW, 2048)
    wheelSpeedEncoder.modelTag = "rwSpeedsEncoder"
    wheelSpeedEncoder.rwSpeedInMsg.subscribeTo(rwSpeedMsg)

You can change the two parameters after the construction:

.. code-block:: python
    :linenos:

    wheelSpeedEncoder.numRW = 4
    wheelSpeedEncoder.clicksPerRotation = 1024

Signal States
~~~~~~~~~~~~~

To set the signal state of one wheel, use ``setSignalState``. The wheel index must be less than ``numRW``.

.. code-block:: python
    :linenos:

    wheelSpeedEncoder.setSignalState(1, encoder.EncoderSignal_Stuck)

To set the signal states of all wheels at the same time, use the ``signalStates`` attribute. The list must contain one state for each wheel.

.. code-block:: python
    :linenos:

    wheelSpeedEncoder.signalStates = [encoder.EncoderSignal_Off] * numRW

Incorrect Values
~~~~~~~~~~~~~~~~

The module rejects an incorrect value when you set it, and keeps the previous value. In Python, an incorrect value causes a ``ValueError``.
In C++, it causes a ``std::invalid_argument`` exception. These values are incorrect:

- A wheel count of zero, or a wheel count that is more than ``RW_EFF_CNT``.
- Zero clicks per rotation.
- A wheel index that is not less than ``numRW``.
- A signal state that is not an ``EncoderSignal`` value.
- A ``signalStates`` list with a size that is not equal to ``numRW``.

If ``rwSpeedInMsg`` is not linked, reset causes an exception. In Python, ``InitializeSimulation`` then stops with a ``RuntimeError``.
