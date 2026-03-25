Executive Summary
-----------------

The ``ImageReaderInterface`` defines a common API for reading and pre-processing images for the
center-of-brightness module. Two concrete implementations are provided:

- **ImageReaderFromFile** — reads images from disk via OpenCV.
- **ImageReaderFromMessage** — reads images from a :ref:`CameraImageMsgPayload` message containing encoded
  image bytes.

Both implementations share the same image-processing pipeline: BGR-to-grayscale conversion, box blur,
binary thresholding, non-zero pixel extraction, and windowed filtering. The output is an array of pixel
coordinates representing the locations of bright pixels within the requested window.


Interface
---------

``ImageReaderInterface`` is an abstract base class with three pure virtual methods:

.. list-table:: Interface Methods
    :widths: 30 70
    :header-rows: 1

    * - Method
      - Description
    * - ``getFullImageSize(cameraId)``
      - Returns the dimensions (width, height) of the most recently read image.
    * - ``getCurrentImageTimeTag(cameraId, previousTimeTag)``
      - Returns the new image time tag if a newer image is available, or 0 otherwise.
    * - ``getImageAsArray(center, windowSize, output)``
      - Reads and processes the image, then writes the pixel coordinates of all bright pixels
        within the specified window into ``output``. Unused slots are filled with the sentinel
        value ``(0, 0)``.


Image Processing Pipeline
--------------------------

Both implementations perform the following steps inside ``getImageAsArray``:

#. **Load image** — from file (``ImageReaderFromFile``) or from an encoded byte buffer in a message
   (``ImageReaderFromMessage``).
#. **Convert to grayscale** — ``cv::cvtColor`` with ``COLOR_BGR2GRAY``.
#. **Box blur** — ``cv::blur`` with a kernel of size ``blurSize × blurSize`` pixels.
#. **Binary threshold** — pixels with intensity above ``pixelThreshold`` are set to 255; all others
   are set to 0.
#. **Find non-zero pixels** — ``cv::findNonZero`` returns the coordinates of all white pixels.
#. **Window filtering** — only coordinates that fall within the window
   ``[center - size/2, center + size/2)`` are kept. The result is written into the output array.

The maximum supported window size is 1024 × 1024 pixels (``kMaxWindowSize = 1,048,576``).


ImageReaderFromFile
---------------

Reads images from disk using ``cv::imread``.

.. list-table:: Configuration
    :widths: 30 15 55
    :header-rows: 1

    * - Setter / Getter
      - Units
      - Description
    * - ``setFileName`` / ``getFileName``
      - \-
      - Path to the image file to read. Must be set before calling ``getImageAsArray``.
    * - ``setBlurSize`` / ``getBlurSize``
      - px
      - Box blur kernel size. Set to 1 for no blurring.
    * - ``setPixelThreshold`` / ``getPixelThreshold``
      - \-
      - Binary threshold value (0–255). Pixels brighter than this are detected.
    * - ``setSaveImages`` / ``getSaveImages``
      - \-
      - When true, saves the raw image to ``saveDir`` on each ``getImageAsArray`` call.
    * - ``setSaveDir`` / ``getSaveDir``
      - \-
      - Output path for saved images.

``getCurrentImageTimeTag`` always returns 1 (file-based reader has no freshness tracking).

Python example::

    reader = centerOfBrightness.ImageReaderFromFile()
    reader.setFileName("/path/to/image.png")
    reader.setBlurSize(5)
    reader.setPixelThreshold(50)


ImageReaderFromMessage
-----------------

Reads images from a :ref:`CameraImageMsgPayload` message. The payload must contain a pointer to
encoded image bytes (JPEG or PNG) and set ``valid = 1``.

.. list-table:: Configuration
    :widths: 30 15 55
    :header-rows: 1

    * - Setter / Getter
      - Units
      - Description
    * - ``setBlurSize`` / ``getBlurSize``
      - px
      - Box blur kernel size. Set to 1 for no blurring.
    * - ``setPixelThreshold`` / ``getPixelThreshold``
      - \-
      - Binary threshold value (0–255). Pixels brighter than this are detected.

.. list-table:: Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - imageInMsg
      - :ref:`CameraImageMsgPayload`
      - Input message containing encoded image bytes and metadata.

``getCurrentImageTimeTag`` reads the message and compares ``timeTag`` against the provided
``previousImageTimeTag``. Returns the new time tag if newer, 0 otherwise.

Python example::

    reader = centerOfBrightness.ImageReaderFromMessage()
    reader.setBlurSize(5)
    reader.setPixelThreshold(50)
    reader.imageInMsg.subscribeTo(cameraImageMsg)
