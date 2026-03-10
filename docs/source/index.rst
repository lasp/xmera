.. _about-bsk:

Xmera: Flexible, Fast, and Reconfigurable Space Simulation Framework
========================================================================

Xmera is an open-source software framework for real-time and faster-than-real-time spacecraft simulation,
designed for both astrodynamics research and mission development. Developed by the Autonomous Vehicle Systems Lab
and the `Laboratory for Atmospheric and Space Physics <https://lasp.colorado.edu>`_ (University of Colorado Boulder), Xmera blends
the flexibility of Python with the execution speed of C/C++.

Get Started
-----------
.. grid::

    .. grid-item-card:: 🚀 Installation Instructions
        :link: install
        :link-type: ref
        :link-alt: Installation instructions

        Get started by installing a Xmera development environment.

    .. grid-item-card:: 📚 Learning Resources
        :link: developer
        :link-type: ref
        :link-alt: Learning resources

        Quickly grasp key concepts.

    .. grid-item-card:: 📖 API and Modules Reference
        :link: api
        :link-type: ref
        :link-alt: API and Modules Reference

        Detailed module documentation.

Introduction
------------

.. sidebar:: Conceptual diagram of simulation framework

    .. image:: _images/static/xmera-concept.svg
       :align: center
       :width: 300px
       :alt: Conceptual diagram of Xmera simulation framework

Xmera allows users to create, configure, and execute spacecraft simulations involving
orbit and attitude dynamics, hardware-in-the-loop scenarios, and Monte-Carlo analyses.
Its modular design supports rapid development and validation of flight software, autonomy solutions,
and mission concepts.

Key features include:

- Real-time and faster-than-real-time simulation capability
- Reconfigurable Python interface over C/C++ core
- Native Monte-Carlo engine for repeatable studies
- Integrated unit-testing and validation support
- Hardware-in-the-loop compatibility
- Cross-platform (Linux, Windows, macOS)

----

Use Cases
---------

Xmera is actively used for:

- Astrodynamics research modeling
- Guidance, estimation, and control algorithm development
- Mission concept support and validation
- Flight software (FSW) development and hardware-in-the-loop (HIL) testing
- Spacecraft autonomy research and AI-based system development
- Post-flight data analysis and validation

----

Who Uses Xmera?
------------------

.. sidebar:: Xmera Users

    - Startup space companies
    - Academic research labs
    - Space mission analysts and contractors
    - Autonomous vehicle systems designers
    - International space organizations

Xmera serves a diverse user community ranging from academic researchers to mission developers and commercial
ventures.

----

Xmera Design Goals
---------------------

At its core, Xmera is designed to balance several challenging goals:

- **Speed:** High-performance simulations via C/C++ back-end
- **Flexibility:** Reconfiguration via Python scripting
- **Analysis Integration:** Built-in numpy/matplotlib support
- **Realtime Capabilities:** Hardware-in-the-loop synchronization
- **Data Control:** Managed communication via message-passing interface (MPI)
- **Cross-Platform Compatibility:** Linux, Windows, and macOS supported
- **Validation and Testing:** Robust, integrated unit and scenario tests
- **Monte-Carlo Simulations:** Bit-for-bit repeatability

----

Related Publications
--------------------

For a complete list of academic publications utilizing Xmera:

📄 :ref:`View full references <publications>`

Highlights include research on attitude control, solar radiation pressure modeling, spacecraft autonomy, and
hardware-in-the-loop simulation architectures.

🤝 :ref:`Support and Contribution <developer>`

----

.. toctree::
   :maxdepth: 1
   :hidden:

   learn/index
   api/index
   install/index
   developer/index
   releases/index
