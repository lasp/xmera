Executive Summary
-----------------

The purpose of this simple aggregate module is to read in a series of navigation messages, and combine their values
into a single output message. The module is able to blend both attitude and translation navigation messages.

The number of input messages is defined through either attMsgCount or transMsgCount.  If either of these
values is zero, then the corresponding output navigation message is populated with zero components.

To select which input message value to use, the module index value must be set for that particular parameter. All
these variables end with Idx. Their default values are 0, indicating that by default the values of the first
navigation message are used. By changing the Idx value the user selects which message content to use for that
variable. This can be set individually for each navigation message variable. In all cases the Idx index must be
less than input navigation message counter attMsgCount or transMsgCount respectively.

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
    * - navAttOutMsg
      - :ref:`NavAttMsgPayload`
      - blended attitude navigation output message
    * - navTransOutMsg
      - :ref:`NavTransMsgPayload`
      - blended translation navigation output message
    * - navAttInMsg
      - :ref:`NavAttMsgPayload`
      - attitude navigation input message stored inside the ``AggregateAttInput`` structure
    * - navTransInMsg
      - :ref:`NavTransMsgPayload`
      - translation navigation input message stored inside the ``AggregateTransInput`` structure

User Guide
----------
The array of messages must be of size 10 or less. The message count should be set to the number of input messages. If
this is 0, then no input messages are read in and a zero output navigation message is produced. In the message count is
larger than 10, then the variable is restricted to 10.

The outline to set up the navAggregate module in Python is as follows:

#. Import the module::

    from Basilisk.fswAlgorithms import navAggregate

#. Create an instantiation::

    module = navAggregate.NavAggregate()

#. Create input navigation message containers::

    navAtt1 = navAggregate.AggregateAttInput()
    navAtt2 = navAggregate.AggregateAttInput()
    navTrans1 = navAggregate.AggregateTransInput()
    navTrans2 = navAggregate.AggregateTransInput()

#. Add the aggregated messages::

    module.attMsgs = [navAtt1, navAtt2]
    module.transMsgs = [navTrans1, navTrans2]

#. Set the number of messages::

    module.setAttMsgCount(2)
    module.setTransMsgCount(2)

#. Set the various navigation information indices::

    # Use sun information from 2nd attitude navigation message, and remaining information from 1st message
    module.setAttTimeIdx(0)
    module.setAttIdx(0)
    module.setRateIdx(0)
    module.setSunIdx(1)
    # Use all translational information from 1st translational navigation message
    module.setTransTimeIdx(0)
    module.setPosIdx(0)
    module.setVelIdx(0)
    module.setDvIdx(0)

#. Subscribe to the messages::

    module.attMsgs[0].navAttInMsg.subscribeTo(navAtt1InMsg)
    module.attMsgs[1].navAttInMsg.subscribeTo(navAtt2InMsg)
    module.transMsgs[0].navTransInMsg.subscribeTo(navTrans1InMsg)
    module.transMsgs[1].navTransInMsg.subscribeTo(navTrans2InMsg)

#. Add model to task::

    sim.AddModelToTask(taskName, module)
