.. _module-testing:

Testing
=======

Inside the module folder there should be a sub-folder called ``_UnitTest``.  It needs to contain one or more python
unit test scripts that check the functionality of the Xmera module.  Each unit test file name must start with
``test_xxxx.py`` such that ``pytest`` recognizes this python file as a test file.

See the module template :ref:`cppModuleTemplate` to find two sample unit test scripts for :ref:`cppModuleTemplate`.
Ideally only the module being tested should be loaded. Any input messages can be created from Python and connected to
the module.
