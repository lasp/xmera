Executive Summary
-----------------

Module computes the unweighted center of brightness of an image within a specified region of interest (ROI).
An ``ImageReaderInterface`` implementation is injected at construction time, decoupling the module from any
particular image source. Two concrete readers are provided: ``ImageReaderFromFile`` (reads from disk) and
``ImageReaderFromMessage`` (reads from a :ref:`CameraImageMsgPayload` message). The reader handles blur,
thresholding, and extraction of non-zero pixel locations within the ROI window. The module then computes the
centroid of those bright pixels, tracks a rolling average of the pixel count, and validates the result against
a configurable brightness-increase threshold.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The module msg connection is set by the
user from python. The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.


.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - opnavCOBOutMsg
      - :ref:`OpNavCOBMsgPayload`
      - Output center of brightness message containing the number of detected pixels,
        center-of-brightness in pixel space, and the rolling average brightness.
    * - centerOfBrightnessDiagnosticOutMsg
      - :ref:`CenterOfBrightnessDiagnosticMsgPayload`
      - Diagnostic output message with validity flags (noPixelTrigger,
        notExceedingBrightnessIncreaseTrigger).
    * - roiInMsg
      - :ref:`RegionOfInterestMsgPayload`
      - Region of interest input message specifying the window center and size.


Detailed Module Description
---------------------------

Architecture
^^^^^^^^^^^^

The module follows the Strategy pattern. At construction time it receives a ``std::shared_ptr`` to an
``ImageReaderInterface``, which provides a common API for reading and pre-processing images regardless of
the source. See :doc:`imageReader/imageReader` for details on the interface and its implementations.

On each ``updateState`` call the module reads the ROI from ``roiInMsg``, delegates image reading and
pixel extraction to the injected reader, and passes the resulting pixel coordinates to the core
algorithm (``CenterOfBrightnessAlgorithm``).

Image Processing
^^^^^^^^^^^^^^^^

Image pre-processing is handled entirely by the ``ImageReaderInterface`` implementation. Each reader
performs the following pipeline:

#. Load the image (from file or message payload).
#. Convert from BGR to grayscale.
#. Apply a box blur with a configurable kernel size (``blurSize``).
#. Apply a binary threshold at ``pixelThreshold``: pixels above the threshold become 255, all others become 0.
#. Extract the coordinates of non-zero pixels via ``cv::findNonZero``.
#. Filter those coordinates to only those falling within the requested window.

The output is a fixed-size array of ``Eigen::Vector2i`` pixel coordinates. Unused slots are filled with
the zero-vector sentinel ``(0, 0)``.

Center of Brightness
^^^^^^^^^^^^^^^^^^^^

The algorithm computes the **unweighted centroid** of all non-zero pixel locations returned by the reader.
Given the set of bright-pixel coordinates :math:`\mathcal{P}` within the window:

.. math::

    \mathcal{P} &= \{ p \in \mathrm{window} \mid I(p) > I_{\min} \} \\
    p_{\mathrm{cob}} &= \frac{1}{|\mathcal{P}|} \sum_{p \in \mathcal{P}} p

where :math:`p_{\mathrm{cob}}` has x and y components written to the output message.

Brightness Validation
^^^^^^^^^^^^^^^^^^^^^

The module tracks the number of bright pixels (``pixelsFound``) as a brightness metric and maintains a
rolling average over the last :math:`N` time steps (``numberOfPointsBrightnessAverage``). On each step it
computes the relative increase of the rolling average:

.. math::

    \Delta_{\mathrm{rel}} = \frac{\bar{B}_{\mathrm{new}} - \bar{B}_{\mathrm{old}}}{\bar{B}_{\mathrm{old}}}

If :math:`\Delta_{\mathrm{rel}}` is below ``relativeBrightnessIncreaseThreshold``, the result is tagged
invalid. If no pixels are found at all, the result is also tagged invalid and the brightness history is
not updated. Diagnostic flags are written to ``centerOfBrightnessDiagnosticOutMsg``.


User Guide
----------
This section outlines the steps needed to set up a Center of Brightness module in Python.

#. Import the module::

    from xmera.fswAlgorithms import centerOfBrightness

#. Create an image reader and configure its processing parameters::

    reader = centerOfBrightness.ImageReaderFromFile()
    reader.setBlurSize(7)
    reader.setPixelThreshold(10)
    reader.setFileName("/path/to/image.png")

   Or, for message-based images::

    reader = centerOfBrightness.ImageReaderFromMessage()
    reader.setBlurSize(7)
    reader.setPixelThreshold(10)
    reader.imageInMsg.subscribeTo(cameraImageMsg)

#. Create the module, passing the reader::

    cobModule = centerOfBrightness.CenterOfBrightness(reader)
    cobModule.modelTag = "centerOfBrightness"

#. Configure the camera ID for the output message::

    cobModule.setCameraID(1)

#. Configure brightness validation parameters (optional)::

    cobModule.setRelativeBrightnessIncreaseThreshold(0.1)
    cobModule.setNumberOfPointsBrightnessAverage(5)

#. Subscribe the module to a region of interest message::

    cobModule.roiInMsg.subscribeTo(roiMsg)

#. Add the module to the simulation task::

    sim.AddModelToTask(taskName, cobModule)

#. The results are available on the output messages ``opnavCOBOutMsg`` and
   ``centerOfBrightnessDiagnosticOutMsg``.
