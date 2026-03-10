.. _module-documentation:

Creating Documentation
======================

Creating the Documentation Page
-------------------------------
Each module should have a plain text RST file in the module folder, along with the ``*.h``, ``*.cpp`` and ``*.i`` files.
This documentation file should provide a description of what the module does, have a table of the input and output
messages used, and provide a user guide.  See :ref:`cppModuleTemplate` for a sample RST module description file that
contains a lot of information on how to document a module.

Testing the Documentation
-------------------------
To test the RST documentation, see :ref:`create-html-docs` on how to build the HTML
documentation in ``docs/build/html``.  From the ``docs`` folder, run::

    make html

The build pipeline automatically discovers module RST files from the source tree
(via ``collect_module_docs.py``), generates Doxygen/Breathe API docs, and runs Sphinx.
No manual configuration is needed.

Building the full Xmera documentation is the recommended way to verify formatting
and cross-references.  To view the result in a browser::

    make view
