.. _messaging-creating-types:
.. _makingModules-2:

Creating New Message Types
==========================

File Locations
--------------

Message payload structs are plain header files stored in a single folder:

``src/architecture/msgPayloadDef/``
    All payloads use C-compatible types so they can be used from both C and
    C++ modules.

Naming Convention
-----------------

- Use **upper camel case** starting with a capital letter.
- The file name must end with ``...MsgPayload.h``.
- Example: ``SomeMsgPayload.h``.

Struct Definition
-----------------

A payload file contains a single ``typedef struct`` with a Doxygen brief and
per-field documentation:

.. code-block:: cpp

    #ifndef SOME_MSG_PAYLOAD_H
    #define SOME_MSG_PAYLOAD_H

    /*! @brief Brief description of what this message contains */
    typedef struct {
        int    variable1;           //!< [units] variable description
        double variable2[3];        //!< [units] variable description
    } SomeMsgPayload;

    #endif

Key points:

- The ``#ifndef`` / ``#define`` include guard prevents double inclusion.
- The struct name **must** match the file name (minus the ``.h`` extension) and
  end with ``MsgPayload``.
- The *payload* is just the data.  The *message object* (``Message<T>``)
  wraps a payload together with a header that tracks write time and author.

Generating Interface Files
--------------------------

After adding a new payload header, register it in
``src/architecture/msgPayloadDef/CMakeLists.txt`` by adding a call to the
CMake helper:

.. code-block:: cmake

    xmera_add_swig_message(SomeMsgPayload)

Then rebuild the project.  CMake will auto-generate the required SWIG
interface files for the new message type.

Custom SWIG Interface Template
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

By default, ``xmera_add_swig_message()`` uses a standard template
(``msgInterfacePy.i.in``) that handles the common case where the payload is a
single self-contained struct.

If a payload references another struct that SWIG must also know about — for
example a nested array-of-structs field — you can supply a custom ``.i``
template via the ``TEMPLATE`` parameter:

.. code-block:: cmake

    xmera_add_swig_message(AccDataMsgF32Payload
                           TEMPLATE
                           "${CMAKE_CURRENT_SOURCE_DIR}/AccDataMsgF32Payload.i")

The custom template adds ``%include`` directives and ``STRUCTASLIST()`` calls
for the nested payload type so that SWIG generates the correct Python bindings.
It uses ``{type}`` and ``{baseDir}`` placeholders that CMake substitutes at
generation time.

A real-world example is located at
``src/fp32-fsw-xmera-2/algorithms/msgPayloadDef/AccDataMsgF32Payload.i``.

Next Steps
----------

- :ref:`messaging-objects` -- how to declare, read, and write messages in a
  module
