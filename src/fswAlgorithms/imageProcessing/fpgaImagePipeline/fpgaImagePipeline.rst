Executive Summary
-----------------

Module simulates the FPGA image processing pipeline of a star-tracker/MAC camera instrument.
It accepts a camera image (either from a message or from a file path) and runs it through
five sequential pipeline stages, publishing every intermediate data product as an output
message so that verification and test code can observe the full computation.

Pipeline stages:

1. **Calibration pre-processing** — per-pixel operations driven by a 16-bit calibration image
   whose upper nibble encodes an op-code and lower 12 bits encode a literal or register reference.
2. **Separable box blur** — a pipelined separable box blur with a configurable kernel size of 5, 7,
   or 9.  ``kernelSize`` rows are processed simultaneously; the kernel is never placed partially
   outside the image, so the border strip of ``(kernelSize-1)/2`` pixels on every side is set to
   zero (no zero-padding or partial sums).
3. **Binary threshold** — pixels above ``threshold`` are set to 1, others to 0, packed MSB-first
   into bytes (``ceil(width*height/8)`` bytes total).
4. **Row/column sums** — counts of above-threshold pixels accumulated per row and per column.
5. **ROI ranking** — the image is divided into square regions of ``roiRegionSize`` pixels per side;
   the top 8 regions by above-threshold pixel count are reported.

Overall data flow::

    ┌──────────┐     ┌─────────────┐     ┌──────────┐     ┌───────────┐     ┌─────────┐
    │  Camera  │     │ Calibration │     │   Box    │     │ Threshold │     │   Row/  │
    │  Image   │────▶│    Pre-     │────▶│   Blur   │────▶│  (1-bit   │────▶│  Col    │────▶ ROI
    │ (12-bit) │     │ processing  │     │ (2-pass) │     │  packed)  │     │  Sums   │
    └──────────┘     └─────────────┘     └──────────┘     └───────────┘     └─────────┘
                           │                   │                 │                │
                     rawImageOutMsg    blurredImageOutMsg  threshImageOutMsg  rowColSumOutMsg


Stage 2: Separable Box Blur
---------------------------

The blur is a separable 2-D box filter that mimics the FPGA streaming pipeline.
``kernelSize`` rows are buffered and processed simultaneously.  The kernel window
never extends outside the image boundary, so only pixels where a full k×k footprint
fits entirely within the image receive a blurred value.  The border strip of
``half = (kernelSize-1)/2`` pixels on every side is set to zero.

**Why separable?**  A k×k box filter applied naively requires k² additions per pixel.
Decomposing it into k independent 1-D horizontal sums (one per buffered row) followed
by a single vertical reduction over those k sums reduces the work to 2k operations per
pixel, matching what the FPGA hardware implements.

Valid output region
~~~~~~~~~~~~~~~~~~~

For an image of width W and height H with kernel size k (``half = (k-1)/2``)::

    ┌──────────────────────────────────────────────────────┐
    │         border (half rows, all zero)                 │
    ├─────┬────────────────────────────────────────┬───────┤
    │  b  │                                        │   b   │
    │  o  │   valid blurred pixels                 │   o   │
    │  r  │   rows  half .. H-1-half               │   r   │
    │  d  │   cols  half .. W-1-half               │   d   │
    │  e  │                                        │   e   │
    │  r  │   (W - k + 1) cols × (H - k + 1) rows │   r   │
    ├─────┴────────────────────────────────────────┴───────┤
    │         border (half rows, all zero)                 │
    └──────────────────────────────────────────────────────┘

Pipeline data flow
~~~~~~~~~~~~~~~~~~

For each row window ``rStart = 0 .. H-k``, ``k`` rows are held in the pipeline simultaneously.
A ``rowSums[k]`` array holds one running 1-D horizontal window sum per row::

    Image rows (rStart = 0, k = 5):

        row 0: [p00 p01 p02 p03 p04 p05 ...]
        row 1: [p10 p11 p12 p13 p14 p15 ...]
        row 2: [p20 p21 p22 p23 p24 p25 ...]   ← processed in parallel
        row 3: [p30 p31 p32 p33 p34 p35 ...]
        row 4: [p40 p41 p42 p43 p44 p45 ...]

    rowSums initialised over columns [0..k-1]:

        rowSums[0] = p00+p01+p02+p03+p04
        rowSums[1] = p10+p11+p12+p13+p14
        rowSums[2] = p20+p21+p22+p23+p24
        rowSums[3] = p30+p31+p32+p33+p34
        rowSums[4] = p40+p41+p42+p43+p44

    Column reduction → blurBuf_ at centre pixel (rStart+half, c+half):

        colSum = rowSums[0]+rowSums[1]+rowSums[2]+rowSums[3]+rowSums[4]
        blurBuf_[(rStart+2)*W + (0+2)] = colSum >> shift

Sliding window (column advance)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After each column position the horizontal window in every row slides right by one pixel:
the pixel entering the right edge of the k-wide window is added; the pixel falling off
the left edge is subtracted::

    Column c=0  →  c=1:

        rowSums[i] += rawBuf_[(rStart+i)*W + (c+k)]   ← add column k
        rowSums[i] -= rawBuf_[(rStart+i)*W + c]        ← remove column 0

    Column position across valid range (W=10, k=5):

        c=0: window covers cols [0..4], output at col 2
        c=1: window covers cols [1..5], output at col 3
        c=2: window covers cols [2..6], output at col 4
        ...
        c=5: window covers cols [5..9], output at col 7  (last valid position)

Row window advance
~~~~~~~~~~~~~~~~~~

After all column positions are processed for a given ``rStart``, the row window slides
down by one row.  Row sums are re-initialised from scratch for the new window::

    rStart=0: rows 0-4 → outputs in row 2  (centre of rows 0-4)
    rStart=1: rows 1-5 → outputs in row 3
    rStart=2: rows 2-6 → outputs in row 4
    ...
    rStart=H-k: rows H-k..H-1 → outputs in row H-1-half

    ┌─────────────────────────────────────────────────────────┐
    │  rStart=0   [ row0 | row1 | row2 | row3 | row4 ]        │
    │                                  ↑ output row 2         │
    ├─────────────────────────────────────────────────────────┤
    │  rStart=1   [ row1 | row2 | row3 | row4 | row5 ]        │
    │                                  ↑ output row 3         │
    ├─────────────────────────────────────────────────────────┤
    │  rStart=2   [ row2 | row3 | row4 | row5 | row6 ]        │
    │                                  ↑ output row 4         │
    └─────────────────────────────────────────────────────────┘

Normalisation
~~~~~~~~~~~~~

The 2-D box sum (sum of k² pixel values) is right-shifted to approximate division by k²::

    blurBuf_[r][c] = colSum >> blurShift(kernelSize)

    kernelSize  blurShift  divisor
    ----------  ---------  -------
         5          1          2
         7          2          4
         9          3          8


Worked example
~~~~~~~~~~~~~~

The following traces the algorithm on a 10×10 image with ``kernelSize = 5``
(``half = 2``, ``blurShift = 1``).  Input pixel values are ``pixel[r][c] = r + c + 1``.

Input image::

    col:   0   1   2   3   4   5   6   7   8   9
    row 0: 1   2   3   4   5   6   7   8   9  10
    row 1: 2   3   4   5   6   7   8   9  10  11
    row 2: 3   4   5   6   7   8   9  10  11  12
    row 3: 4   5   6   7   8   9  10  11  12  13
    row 4: 5   6   7   8   9  10  11  12  13  14
    row 5: 6   7   8   9  10  11  12  13  14  15
    row 6: 7   8   9  10  11  12  13  14  15  16
    row 7: 8   9  10  11  12  13  14  15  16  17
    row 8: 9  10  11  12  13  14  15  16  17  18
    row 9: 10  11  12  13  14  15  16  17  18  19

``numOutRows = numOutCols = 10 - 5 + 1 = 6``; valid outputs written to rows 2-7, cols 2-7.

**rStart = 0 (pipeline window: rows 0-4)**

Initialise ``rowSums[5]`` over columns 0-4::

    rowSums[0] = 1+2+3+4+5  = 15    (row 0, cols 0-4)
    rowSums[1] = 2+3+4+5+6  = 20    (row 1, cols 0-4)
    rowSums[2] = 3+4+5+6+7  = 25    (row 2, cols 0-4)
    rowSums[3] = 4+5+6+7+8  = 30    (row 3, cols 0-4)
    rowSums[4] = 5+6+7+8+9  = 35    (row 4, cols 0-4)

Column ``c = 0`` — kernel footprint rows 0-4, cols 0-4 (brackets mark the active window)::

    col:   0   1   2   3   4   5  ...
    row 0: [ 1   2   3   4   5 ] 6  ...
    row 1: [ 2   3   4   5   6 ] 7  ...
    row 2: [ 3   4   5   6   7 ] 8  ...   (centre row, rStart+half = 2)
    row 3: [ 4   5   6   7   8 ] 9  ...
    row 4: [ 5   6   7   8   9 ]10  ...

colSum = 15+20+25+30+35 = 125.
``blurBuf_`` at centre pixel (2, 2): 125 >> 1 = **62**.

Advance window — add column 5, subtract column 0::

    rowSums[0] += pixel[0][5] - pixel[0][0] = 6 - 1 = +5  --> 20
    rowSums[1] += pixel[1][5] - pixel[1][0] = 7 - 2 = +5  --> 25
    rowSums[2] += pixel[2][5] - pixel[2][0] = 8 - 3 = +5  --> 30
    rowSums[3] += pixel[3][5] - pixel[3][0] = 9 - 4 = +5  --> 35
    rowSums[4] += pixel[4][5] - pixel[4][0] = 10- 5 = +5  --> 40

Column ``c = 1`` — kernel footprint rows 0-4, cols 1-5::

    col:   0  [1   2   3   4   5 ] 6  ...
    row 0: 1  [2   3   4   5   6 ] 7  ...
    row 1: 2  [3   4   5   6   7 ] 8  ...
    row 2: 3  [4   5   6   7   8 ] 9  ...   (centre row)
    row 3: 4  [5   6   7   8   9 ]10  ...
    row 4: 5  [6   7   8   9  10 ]11  ...

colSum = 20+25+30+35+40 = 150.
``blurBuf_`` at centre pixel (2, 3): 150 >> 1 = **75**.

Columns ``c = 2..5`` follow the same pattern; each advance adds +5 to every ``rowSum``
because the input gradient is uniform (+1 per pixel).

**rStart = 1 (pipeline window: rows 1-5)**

``rowSums`` is re-initialised from rows 1-5 over columns 0-4.  Output pixels are
written to row ``rStart + half = 3``.  The same column sliding proceeds across ``c = 0..5``.

Row pipeline progression (all ``rStart`` steps, at ``c = 0``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The table below shows the ``rowSums`` contribution from every row that is active in
the k-row window for each ``rStart`` step.  Values shown are the 1-D horizontal sums
over cols 0–4.  Rows outside the current window are blank.  The window slides one
row downward each step; the colSum and blurred output follow at the bottom::

    rStart:         0      1      2      3      4      5
               ┌──────────────────────────────────────────
    row  0:    │  [15]
    row  1:    │  [20]  [20]
    row  2:    │  [25]  [25]  [25]
    row  3:    │  [30]  [30]  [30]  [30]
    row  4:    │  [35]  [35]  [35]  [35]  [35]
    row  5:    │        [40]  [40]  [40]  [40]  [40]
    row  6:    │              [45]  [45]  [45]  [45]
    row  7:    │                    [50]  [50]  [50]
    row  8:    │                          [55]  [55]
    row  9:    │                                [60]
               └──────────────────────────────────────────
    colSum:        125    150    175    200    225    250
    out row:         2      3      4      5      6      7
    blurBuf:        62     75     87    100    112    125

The staircase shape traces the five-row window sweeping from the top of the image to
the bottom.  Each column of the table is one ``rStart`` iteration; each row of the
table is one ``rowSums[i]`` entry.  The output row advances by one for every step
because the centre of the window is always at ``rStart + half``.

Complete ``blurBuf_`` output (0 = border pixel, never written)::

    col:   0    1    2    3    4    5    6    7    8    9
    row 0: 0    0    0    0    0    0    0    0    0    0
    row 1: 0    0    0    0    0    0    0    0    0    0
    row 2: 0    0   62   75   87  100  112  125    0    0
    row 3: 0    0   75   87  100  112  125  137    0    0
    row 4: 0    0   87  100  112  125  137  150    0    0
    row 5: 0    0  100  112  125  137  150  162    0    0
    row 6: 0    0  112  125  137  150  162  175    0    0
    row 7: 0    0  125  137  150  162  175  187    0    0
    row 8: 0    0    0    0    0    0    0    0    0    0
    row 9: 0    0    0    0    0    0    0    0    0    0

Manual spot-check — ``blurBuf_`` at (3, 7), kernel at rows 1-5, cols 5-9::

    sum = (7+8+9+10+11) + (8+9+10+11+12) + (9+10+11+12+13)
        + (10+11+12+13+14) + (11+12+13+14+15)
        =  45 + 50 + 55 + 60 + 65
        = 275
    275 >> 1 = 137  (matches blurBuf_[3][7] above)


Message Connection Descriptions
--------------------------------

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - imageInMsg
      - :ref:`CameraImageMsgPayload`
      - Optional camera image input message; the ``imagePointer`` is cast to ``uint16_t*``
        (12-bit values in lower bits, one element per pixel).
    * - rawImageOutMsg
      - :ref:`FpgaRawImageMsgPayload`
      - Calibration-preprocessed image; ``imagePointer`` points to the module's internal buffer.
    * - blurredImageOutMsg
      - :ref:`FpgaRawImageMsgPayload`
      - Box-blurred image; ``imagePointer`` points to the module's internal buffer.
    * - threshImageOutMsg
      - :ref:`FpgaThreshImageMsgPayload`
      - 1-bit packed binary threshold result; ``imagePointer`` points to the module's internal buffer.
    * - rowColSumOutMsg
      - :ref:`FpgaRowColSumMsgPayload`
      - Per-row and per-column above-threshold pixel counts.
    * - roiOutMsg
      - :ref:`FpgaRoiMsgPayload`
      - Top-8 ROI regions sorted descending by above-threshold pixel count.
    * - configOutMsg
      - :ref:`FpgaPipelineConfigMsgPayload`
      - Snapshot of the active pipeline configuration.


User Guide
----------

#. Import the module::

    from xmera.fswAlgorithms import fpgaImagePipeline

#. Instantiate and configure::

    pipe = fpgaImagePipeline.FpgaImagePipeline()
    pipe.ModelTag = "fpgaPipeline"
    pipe.setImageWidth(4096)
    pipe.setImageHeight(3000)
    pipe.setKernelSize(5)          # 5, 7, or 9
    pipe.setThreshold(200)
    pipe.setRoiRegionSize(64)      # 64, 128, or 256

#. Connect an image source (choose one)::

    # Option A — disk file (useful for unit testing)
    pipe.setImageFileName("/path/to/image.tiff")

    # Option B — live message from camera emulator
    pipe.imageInMsg.subscribeTo(cameraModule.imageOutMsg)

#. Optionally enable calibration::

    pipe.setCalibEnabled(True)
    pipe.setCalibImageFile("/path/to/calib.tiff")
    pipe.setCalibRegA(100)   # register values for op-codes 0x1/0x6/0xb

#. Optionally enable intermediate image saving::

    pipe.setSaveImages(True)
    pipe.setSaveDir("/tmp/fpga_out")

#. Add to simulation task::

    sim.AddModelToTask(taskName, pipe)

#. Access output messages or internal buffers from Python::

    # Read output message fields
    rawMsg = pipe.rawImageOutMsg.read()
    print(rawMsg.width, rawMsg.height)
    # Access internal pixel values directly (for testing)
    pixel = pipe.getRawPixel(row * width + col)
    above = pipe.getThreshBit(row * width + col)

Note: internal buffer accessors are available for testing only and expose raw pointers
that are valid only for the lifetime of the module.
