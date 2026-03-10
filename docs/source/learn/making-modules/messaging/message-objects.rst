.. _messaging-objects:
.. _cppModules-2:

Using Message Objects
=====================

The message template classes are defined in ``src/architecture/messaging/messaging.h``.
Include this header in any module that reads or writes messages.

Input Messages (``ReadFunctor<T>``)
-----------------------------------

Declaring an Input
^^^^^^^^^^^^^^^^^^

Add a public member in the module header:

.. code-block:: cpp

    ReadFunctor<SomeMsgPayload> someInMsg;

Reading a Message
^^^^^^^^^^^^^^^^^

Call the read functor like a function.  It returns a deep copy of the latest
payload:

.. code-block:: cpp

    SomeMsgPayload localBuffer = this->someInMsg();

.. note::

    The local buffer does **not** need to be zero-initialised -- the read
    functor overwrites it entirely.  A compile-time error is raised if the
    buffer type does not match the message type.

Helper Methods
^^^^^^^^^^^^^^

``isLinked()``
    Returns ``true`` if the input is connected to an output message.

``isWritten()``
    Returns ``true`` if the connected output has been written to at least once.

``timeWritten()``
    Returns the simulation time (nanoseconds, ``uint64_t``) at which the
    connected message was last written.

``moduleID()``
    Returns the ``int64_t`` ID of the module that wrote the message.  C/C++
    module IDs are strictly positive; Python module IDs are strictly negative.

Example:

.. code-block:: cpp

    if (this->someInMsg.isLinked()) {
        SomeMsgPayload data = this->someInMsg();
    }

Output Messages (``Message<T>``)
---------------------------------

Declaring an Output
^^^^^^^^^^^^^^^^^^^

Add a public member in the module header:

.. code-block:: cpp

    Message<SomeMsgPayload> someOutMsg;

Writing a Message
^^^^^^^^^^^^^^^^^

Fill a zero-initialised local buffer and call ``write()``:

.. code-block:: cpp

    SomeMsgPayload buf{};
    buf.dataVector[0] = 1.0;
    this->someOutMsg.write(&buf, this->moduleID, currentSimNanos);

.. note::

    Always zero-initialise the output buffer with ``{}`` to avoid writing
    uninitialised values.  A compile-time error is raised if the buffer type
    does not match the message type.

Helper Methods
^^^^^^^^^^^^^^

``isLinked()``
    Returns ``true`` if any input message (including a ``recorder()``) is
    subscribed to this output.

``addSubscriber()``
    Returns a ``ReadFunctor<T>`` that can read this output message.

``addAuthor()``
    Returns the write functor, granting write access to the message object.

Vectors of Input Messages
-------------------------

Declare a vector of read functors and a corresponding private buffer vector:

.. code-block:: cpp

    public:
        std::vector<ReadFunctor<SomeMsgPayload>> moreInMsgs;
    private:
        std::vector<SomeMsgPayload> moreInMsgsBuffer;

Use a public method to let the user add input messages:

.. code-block:: cpp

    void SomeModule::addMsgToModule(Message<SomeMsgPayload> *tmpMsg)
    {
        this->moreInMsgs.push_back(tmpMsg->addSubscriber());

        SomeMsgPayload emptyBuf;
        this->moreInMsgsBuffer.push_back(emptyBuf);
    }

Vectors of Output Messages
---------------------------

Output message vectors use pointers because each ``Message`` object must
persist after the helper method returns:

.. code-block:: cpp

    public:
        std::vector<Message<SomeMsgPayload>*> moreOutMsgs;

Create output messages dynamically:

.. code-block:: cpp

    Message<SomeMsgPayload> *msg = new Message<SomeMsgPayload>;
    this->moreOutMsgs.push_back(msg);

Free the memory in the destructor:

.. code-block:: cpp

    SomeModule::~SomeModule()
    {
        for (auto *msg : this->moreOutMsgs) {
            delete msg;
        }
    }

See :ref:`eclipse` for a real-world example where each spacecraft state input
message is paired with a corresponding eclipse output message.

Best Practices
--------------

- Always zero-initialise output buffers with ``{}`` before writing.
- Check ``isLinked()`` in ``reset()`` for required input messages and log an
  error if the message is not connected.
- Avoid dynamic allocation in tight loops -- prefer fixed-size arrays where
  possible.

Next Steps
----------

- :ref:`messaging-overview` -- high-level introduction to the messaging system
- :ref:`messaging-creating-types` -- how to define new payload structs
