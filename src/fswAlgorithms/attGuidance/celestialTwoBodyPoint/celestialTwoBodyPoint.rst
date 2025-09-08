Executive Summary
-----------------

This module point one body-fixed axis towards a primary celestial object. The secondary goal is to point a second
body-fixed axis towards another celestial object.

For example, the goal is to point the sensor towards the center of a planet while doing the best to keep the solar
panel normal point at the sun.

The module assumes that the acceleration of the spacecraft with respect to the planets is negligible.

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
    * - attRefOutMsg
      - :ref:`AttRefMsgPayload`
      - attitude reference output message
    * - transNavInMsg
      - :ref:`NavTransMsgPayload`
      - spacecraft translation motion input message
    * - celBodyInMsg
      - :ref:`EphemerisMsgPayload`
      - primary celestial body information input message
    * - secCelBodyInMsg
      - :ref:`EphemerisMsgPayload`
      - (optional) secondary celestial body information


Reference Frame Definition
--------------------------

This module computes a reference whose aim is to track the center of a
primary target, e.g. pointing the communication antenna at the Earth,
at the same time of trying to meet a secondary constraint as best as
possible, e.g. a solar panel normal axis pointing the closest in the
direction of the Sun. It is important to note that two pointing
conditions in a three-dimensional space compose an overdetermined
problem. Thus, the main constraint is always priorized over the
secondary one so the former can always be met.
Figure :ref:`1 <#fig:fig1>`__ shows the case where Mars is the primary
celestial body and the Sun is the secondary one. Note that the origin
of the desired reference frame :math:`\mathcal{R}` is the position of
the spacecraft.

.. _fig1_celestialTwoBodyPoint:
.. figure:: _Documentation/Figures/fig1.pdf
    :align: center
   name: fig:fig1

Illustration of the restricted two-body pointing reference frame
:math:`\mathcal{R}:\{ \hat{\mathbf r}_{1},\hat{\mathbf r}_{1}, \hat{\mathbf r}_{2} \}`
and the inertial frame
:math:`\mathcal{N}:\{ \hat{\mathbf n}_{1},\hat{\mathbf n}_{1}, \hat{\mathbf n}_{2} \}`.

Assuming knowledge of the position of the spacecraft
:math:`\mathbf{r}_{B/N}` and the involved celestial bodies,
:math:`\mathbf{R}_{P1}` and :math:`\mathbf{R}_{P2}` (all of them relative to the
inertial frame :math:`\mathcal{N}` and expressed in inertial frame
components), the relative position of the celestial bodies with respect
to the spacecraft is obtained by simple subtraction:

.. math::

   \begin{align}
            \mathbf{R}_{P1} =\mathbf{R}_{P} - \mathbf{r}_{B/N} \\
            \mathbf{R}_{P2} =\mathbf{R}_{S} - \mathbf{r}_{B/N}
       \end{align}

In analogy, the inertial derivatives of these position vectors are
obtained:

.. math::

   \begin{align}
            \mathbf{v}_{P1} &=\mathbf{v}_{P} - \mathbf{v}_{B/N} \\
            \mathbf{v}_{P2} &=\mathbf{v}_{S} - \mathbf{v}_{B/N} \\
            \mathbf{a}_{P1} &=\mathbf{a}_{P} - \mathbf{a}_{B/N} \\
            \mathbf{a}_{P2} &=\mathbf{a}_{S} - \mathbf{a}_{B/N}
       \end{align}

Note that, while the documentation includes the full derivation including the acceleration of the spacecraft with
respect to the planets, the module assumes that :math:`\mathbf{a}_{P1} = \mathbf{a}_{P2} = 0`.

The normal vector :math:`\mathbf{R}_{n}` of the plane defined by
:math:`\mathbf{R}_{P1}` and :math:`\mathbf{R}_{P2}` is computed through:

.. math:: \mathbf R_{n} =\mathbf{R}_{P1} \times \mathbf{R}_{P2}

The inertial time derivative of :math:`\mathbf{R}_n` is found using the
chain differentiation rule:

.. math:: \mathbf {v}_{n} = \mathbf{v}_{P1} \times \mathbf{R}_{P2} + \mathbf{R}_{P1} \times \mathbf{v}_{P2}

And the second time derivative:

.. math:: \mathbf {a}_{n} = \mathbf{a}_{P1} \times \mathbf{R}_{P2} + \mathbf{R}_{P1} \times \mathbf{a}_{P2}  + 2 \mathbf{v}_{P1} \times \mathbf{v}_{P2}

Special Case: No Secondary Constraint Applicable
================================================

If there is no incoming message with a secondary celestial body pointing
condition or if the constrain is not valid, an artificial
three-dimensional frame is defined instead. Note that a single condition
pointing leaves one degree of freedom, hence standing for an
underdetermined attitude problem. A secondary constrain is considered to
be invalid if the angle between :math:`\mathbf{R}_{P1}` and
:math:`\mathbf{R}_{P2}` is, in absolute value, minor than a set threshold.
This could be the case where the primary and secondary celestial bodies
are aligned as seen by the spacecraft. In such situation, the primary
pointing axis would already satisfy both the primary and the secondary
constraints.

Since the main algorithm of this module, which is developed in the
following sections, assumes two conditions, the second one is
arbitrarily set as following:

.. math:: \mathbf{R}_{P2} = \mathbf{R}_{P1} \times \mathbf{v}_{P1} \equiv  \mathbf{h}_{P1}

By setting the secondary constrain to have the direction of the angular
momentum vector :math:`\mathbf{h}_{P1}`, it is assured that it will always
be valid (:math:`\mathbf{R}_{P1}` and :math:`\mathbf{R}_{P2}` are now
perpendicular). The first and second time derivatives are steadily
computed:

.. math:: \mathbf{v}_{P2} =  \mathbf{R}_{P1} \times \mathbf{a}_{P1}

.. math:: \mathbf{a}_{P2} =  \mathbf{v}_{P1} \times \mathbf{a}_{P1}

.. _reference-frame-definition-1:

Reference Frame Definition
==========================

As illustrated in Figure `1 <#fig:fig1>`__, the base vectors of the
desired reference frame :math:`\mathcal{R}` are defined as following:

.. math::

   \begin{align}
           \hat{\mathbf r}_{1} &= \frac{{\mathbf R}_{P1}} {|{\mathbf R}_{P1}|} \\
           \hat{\mathbf r}_{3} &= \frac{{\mathbf R}_{n}}{|{\mathbf R}_{n}|} \\
           \hat{\mathbf r}_{2} &=  \hat{\mathbf r}_{3} \times \hat{\mathbf r}_{1}
       \end{align}

Since the position vectors are given in terms of inertial
:math:`\mathcal{N}`-frame components, the DCM from the inertial frame
:math:`\mathcal{N}` to the desired reference frame :math:`\mathcal{R}`
is:

.. math::

   [RN] = \left[\begin{matrix}
           {}^{N}{ \hat{\mathbf r}_{1}^{T} } \\
           {}^{N}{ \hat{\mathbf r}_{2}^{T} } \\
           {}^{N}{ \hat{\mathbf r}_{3}^{T} }
        \end{matrix}\right]

Base Vectors Time Derivatives
=============================

The first and second time derivatives of the base vectors that compound
the reference frame :math:`\mathcal{R}` are needed in the following
sections to compute the reference angular velocity and acceleration.
Several lines of algebra lead to the following sets:

.. math::

   \begin{align}
           \dot{\hat{\mathbf{r}}}_1 &= ([I_{3\times3}] - {\hat{\mathbf{r}}_1}{\hat{\mathbf{r}}_1}^T)  \frac{{\mathbf V}_{P1}} {|{\mathbf R}_{P1}|} \\
           \dot{\hat{\mathbf{r}}}_3 &= ([I_{3\times3}] - \hat{\mathbf{r}}_3 \hat{\mathbf{r}}_3^T)  \frac{{\mathbf v}_{n}} {|{\mathbf R}_{n}|} \\
           \dot{\hat{\mathbf{r}}}_2 &= \dot{\hat{\mathbf{r}}}_3 \times \hat{\mathbf{r}}_1 +  \hat{\mathbf{r}}_3  \times \dot{\hat{\mathbf{r}}}_1
       \end{align}

.. math::

   \begin{align}
           \ddot{\hat{\mathbf{r}}}_1 &= \frac{1}{|{\mathbf R}_{P1}|}
           (
           ([I_{3\times3}] - {\hat{\mathbf{r}}_1}{\hat{\mathbf{r}}_1}^T)  \mathbf{a}_{P1} -
           2\dot{\hat{\mathbf{r}}}_1 (\hat{\mathbf{r}}_1 \cdot \mathbf{v}_{P1}) -
           \hat{\mathbf{r}}_1 (\dot{\hat{\mathbf{r}}}_1 \cdot \mathbf{v}_{P1})
           ) \\
           \ddot{\hat{\mathbf{r}}}_3 &= \frac{1}{|{\mathbf R}_{n}|}
           (
           ([I_{3\times3}] - {\hat{\mathbf{r}}_3}{\hat{\mathbf{r}}_3}^T)  \mathbf{a}_{n} -
           2\dot{\hat{\mathbf{r}}}_3 (\hat{\mathbf{r}}_3 \cdot \mathbf{v}_{n}) -
           \hat{\mathbf{r}}_3 (\dot{\hat{\mathbf{r}}}_3 \cdot \mathbf{v}_{n})
           ) \\
           \ddot{\hat{\mathbf{r}}}_2 &= \ddot{\hat{\mathbf{r}}}_3 \times \hat{\mathbf{r}}_1 +  \hat{\mathbf{r}}_3  \times \ddot{\hat{\mathbf{r}}}_1 + 2 \dot{\hat{\mathbf{r}}}_3 \times \dot{\hat{\mathbf{r}}}_1
       \end{align}

Angular Velocity and Acceleration Descriptions
==============================================

Developing some more mathematics, the following elegant expressions of
:math:`\mathbf\omega_{R/N}` and :math:`\dot{\mathbf\omega}_{R/N}` are found:

.. math::

   \begin{align}
           \mathbf\omega_{R/N} \cdot \hat{\mathbf r}_{1}  = \hat{\mathbf r}_{3} \cdot \dot{\hat{\mathbf r}}_{2}  \\
           \mathbf\omega_{R/N} \cdot \hat{\mathbf r}_{2} = \hat{\mathbf r}_{1} \cdot \dot{\hat{\mathbf r}}_{3}\\
           \mathbf\omega_{R/N} \cdot \hat{\mathbf r}_{3} = \hat{\mathbf r}_{2} \cdot \dot{\hat{\mathbf r}}_{1}
       \end{align}

.. math::

   \begin{align}
           \dot{\mathbf\omega}_{R/N} \cdot \hat{\mathbf r}_{1} &=
           \dot{\hat{\mathbf r}}_{3} \cdot \dot{\hat{\mathbf r}}_{2} + \hat{\mathbf r}_{3} \cdot \ddot{\hat{\mathbf r}}_{2} -  \mathbf\omega_{R/N} \cdot \dot{\hat{\mathbf r}}_{1}
           \\
           \dot{\mathbf\omega}_{R/N} \cdot \hat{\mathbf r}_{2} &=
            \dot{\hat{\mathbf r}}_{1} \cdot \dot{\hat{\mathbf r}}_{3} + \hat{\mathbf r}_{1} \cdot \ddot{\hat{\mathbf r}}_{3} -  \mathbf\omega_{R/N} \cdot \dot{\hat{\mathbf r}}_{2}
           \\
           \dot{\mathbf\omega}_{R/N} \cdot \hat{\mathbf r}_{3} &=
           \dot{\hat{\mathbf r}}_{2} \cdot \dot{\hat{\mathbf r}}_{1} + \hat{\mathbf r}_{2} \cdot \ddot{\hat{\mathbf r}}_{1} -  \mathbf\omega_{R/N} \cdot \dot{\hat{\mathbf r}}_{3}
       \end{align}

Note that :math:`\mathbf\omega_{R/N} \cdot \hat{\mathbf r}_{1}` is the first
component of the angular velocity of the reference with respect to the
inertial expressed in reference frame components. Hence,

.. math::

   \mathbf\omega_{R/N}= {}^{R}{
           \left[\begin{matrix}
               \mathbf\omega_{R/N} \cdot \hat{\mathbf r}_{1} \\
               \mathbf\omega_{R/N} \cdot \hat{\mathbf r}_{2}  \\
               \mathbf\omega_{R/N} \cdot \hat{\mathbf r}_{3}
           \end{matrix}\right]
       }

Similarly for the angular acceleration:

.. math::

   \mathbf{\dot\omega}_{R/N} = {}^{R}{
           \left[\begin{matrix}
               \mathbf{\dot\omega}_{R/N} \cdot \hat{\mathbf r}_{1} \\
               \mathbf{\dot\omega}_{R/N} \cdot \hat{\mathbf r}_{2}  \\
               \mathbf{\dot\omega}_{R/N} \cdot \hat{\mathbf r}_{3}
           \end{matrix}\right]
       }

Eventually, in inertial frame components:

.. math::

   \begin{align}
           {}^{N} {\mathbf\omega_{R/N}} &= [RN] \textrm{ } {}^{R} {\mathbf\omega_{R/N}}
           \\
           {}^{N} {\mathbf{\dot\omega}_{R/N}} &= [RN]  \textrm{ } {}^{R} {\mathbf{\dot\omega}_{R/N}}
       \end{align}
