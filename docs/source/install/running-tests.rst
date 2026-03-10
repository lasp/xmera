.. toctree::
   :hidden:

.. _running_tests:

Running Project Tests
---------------------

The project employs two testing frameworks, specifically `Pytest <https://pytest.org>`__ for python
executed tests and `Google Test <https://github.com/google/googletest>`__ for C/C++ executed tests.

To run all tests execute the following from the project root directory

.. code-block:: console

    python run_all_test.py

To run only the python test use the following commands.

.. code-block:: console

            cd src
            pytest

To run only the C/C++ tests use

.. tab-set::

    .. tab-item:: Linux/Windows

        .. code-block:: console

            cd src
            ctest

    .. tab-item:: macOS

        .. code-block:: console

            cd dist3
            ctest -C <Release or Debug>

Running Tests In Parallel
=========================

One can distribute python tests across multiple processes. This is achieved with the ``pytest-xdist``
package using

.. code-block:: console

   pip3 install pytest-xdist

After installing this package you can now pytest such that it distribute tests across multi-processes.
``pytest`` for the same number of processes as processors on the host machine using

.. code-block:: console

   python3 -m pytest -n auto

or replace `auto` with the number of processors (virtual or otherwise) you'd like to dedicate as test executions
processes.

Generating HTML Report of the ``pytest`` Results
================================================

If you want to use ``pytest`` to generate a validation HTML report,
then the ``pytest-html`` package must be installed

.. code-block:: console

   pip3 install pytest-html

When running ``pytest`` from the terminal, add the ``--html`` argument followed by the path to where
to generate the report html folder.  It is recommended to put this inside a folder as HTML
support folder will be created

.. code-block:: console

    pytest --html report/report.html
