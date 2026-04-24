Executive Summary
-----------------

``RegionsOfInterestPrune`` consumes the row/column above-threshold pixel sums produced by
``fpgaImagePipeline`` and outputs up to ``ROI_CANDIDATES_MAX`` bounding-box candidates sorted
by estimated pixel count.  It sits immediately downstream of ``fpgaImagePipeline`` and feeds
downstream tracking or centroiding stages.

The module is split into two classes:

* ``RegionsOfInterestPruneAlgorithm`` — pure C++, no Basilisk dependencies.  Accepts raw
  ``uint16_t`` row/col sum arrays and returns a ``RoiCandidatesMsgPayload``.
* ``RegionsOfInterestPrune`` — thin Basilisk adapter.  Reads ``FpgaRowColSumMsgPayload``,
  delegates computation to ``RegionsOfInterestPruneAlgorithm``, and writes the result to
  ``candidatesOutMsg``.

Pipeline stages::

    rowColSumInMsg
         │
         ▼
    1. Find spans   ──  contiguous non-zero runs in rowSums / colSums; accumulate per-span sums
         │
         ▼
    2. Pre-filter   ──  top maxRowSpans row spans, top maxColSpans col spans (by accumulator)
         │
         ▼
    3. Cross-product ─  bounding boxes; count = min(R[k], C[l]) per box
         │
         ▼
    4. Sort & truncate  sort by count desc, keep ≤ ROI_CANDIDATES_MAX
         │
         ▼
    candidatesOutMsg


Algorithm Details
-----------------

**Step 1 — span detection and accumulation**

A *span* is a maximal contiguous run of non-zero entries in a 1-D sum array.  Each row span
``(r, h)`` and col span ``(c, w)`` defines the start and length of a group of image rows or
columns that contain at least one above-threshold pixel.  The per-span accumulator sum ``R[k]``
(or ``C[l]``) is computed in the same forward pass as span detection.

**Step 2 — pre-filter safeguard**

In the worst case the cross-product of all row spans and all col spans is unbounded.  To cap
computation, the module keeps only the top ``maxRowSpans`` row spans (by their row-sum
accumulator ``R[k]``) and the top ``maxColSpans`` col spans (by ``C[l]``).  The cross-product
is then at most ``maxRowSpans × maxColSpans`` candidates regardless of image size.

Because the true rank-1 candidate lies at the intersection of ``argmax(R)`` and ``argmax(C)``,
it is always preserved by this filter.

**Step 3 — cross-product and pixel count estimation**

Every combination of a filtered row span ``k`` and a filtered col span ``l`` defines a
bounding box candidate ``(r, h, c, w)`` where ``r`` / ``h`` are the start row and height from
span ``k``, and ``c`` / ``w`` are the start column and width from span ``l``.  The estimated
pixel count for each box is:

* ``R[k]`` = sum of ``rowSums`` over row span ``k`` — overcounts pixels outside col span ``l``
* ``C[l]`` = sum of ``colSums`` over col span ``l`` — overcounts pixels outside row span ``k``
* ``count = min(R[k], C[l])`` — tightest upper bound obtainable from 1-D projections alone

**Step 4 — sort and truncate**

All candidates from Step 3 are sorted by ``count`` in descending order.  If the number of
candidates exceeds ``ROI_CANDIDATES_MAX`` the tail is dropped.  The final list is packed into
``RoiCandidatesMsgPayload`` with ``candidates[0]`` as rank-1 (highest count).


Message Connection Descriptions
--------------------------------

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - rowColSumInMsg
      - :ref:`FpgaRowColSumMsgPayload`
      - Per-row and per-column above-threshold pixel counts from ``fpgaImagePipeline``.
    * - candidatesOutMsg
      - :ref:`RoiCandidatesMsgPayload`
      - Up to ``ROI_CANDIDATES_MAX`` bounding-box candidates sorted by estimated pixel count.
        ``candidates[0]`` is rank-1 (highest count), ``candidates[1]`` is rank-2, etc.


User Guide
----------

#. Import the module::

    from xmera.fswAlgorithms import regionsOfInterestPrune

#. Instantiate and configure::

    pruner = regionsOfInterestPrune.RegionsOfInterestPrune()
    pruner.ModelTag = "roiPrune"

    # Optional: tune the pre-filter safeguard (defaults: 3 each)
    pruner.setMaxRowSpans(10)
    pruner.setMaxColSpans(10)

#. Connect to the upstream pipeline::

    pruner.rowColSumInMsg.subscribeTo(pipeline.rowColSumOutMsg)

#. Add to simulation task::

    sim.AddModelToTask(taskName, pruner)

#. Read results::

    n     = pruner.getNumCandidates()        # number of candidates returned (≤ ROI_CANDIDATES_MAX)
    row   = pruner.getCandidateRow(0)        # top-ranked candidate
    col   = pruner.getCandidateCol(0)
    h     = pruner.getCandidateHeight(0)
    w     = pruner.getCandidateWidth(0)
    count = pruner.getCandidateCount(0)      # estimated above-threshold pixel count


Unit Tests
----------

All tests reside in ``_UnitTest/test_regionsOfInterestPrune.py``.

.. list-table::
    :widths: 30 70
    :header-rows: 1

    * - Test
      - Description
    * - ``test_message_connection``
      - Verifies ``rowColSumInMsg`` links correctly and the received payload carries the
        expected image dimensions after one simulation step.
    * - ``test_step2_rank1_only``
      - Single 5×5 bright block (M=1): checks exactly one candidate is output with count=1.
    * - ``test_step2_rank1_and_rank2``
      - 13×13 and 5×5 blocks sharing a col span: verifies rank-1 and rank-2 bounding boxes
        and positions.
    * - ``test_step2_rank2_by_count``
      - Three col-aligned blocks (13×13, 9×9, 5×5): confirms candidates are sorted by
        estimated count (81, 25, 1) so rank-2 is the 9×9 block, not the 5×5 block.
    * - ``test_pruning``
      - End-to-end integration on ``pia_958_830.tiff`` through the full
        ``fpgaImagePipeline`` → ``regionsOfInterestPrune`` chain.  Parametrized over
        ``row_col_span`` ∈ {2, 3, 4} (passed to ``setMaxRowSpans`` / ``setMaxColSpans``).
        The threshold is auto-computed from the 95th percentile of the image blur map.
        Skipped if the test image is not found.  Writes two output files:

        * ``test_pruning_output.png`` — colour-annotated image with all candidates in yellow,
          rank-1 in red labelled ``R1 (<count>)``, rank-2 in blue labelled ``R2 (<count>)``.
        * ``test_pruning_threshold.png`` — binary mask of above-threshold pixels output by
          ``fpgaImagePipeline`` (above-threshold → 255, below → 0).
