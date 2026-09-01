"""
Unit tests for fpgaImagePipeline — one smoke test plus one test per computation
function in updateState():

    test_smoke_small_image        — end-to-end: verifies rawImageOutMsg is published
    applyBlurAndThreshold()  →  test_apply_blur_and_threshold
    computeRowColSums()      →  test_compute_row_col_sums
    computeRoi()             →  test_compute_roi
    blur test for single impulse → getBlurPixel
    message chaining — rawImageOutMsg fields are correctly written and readable.

All tests use a fully synthetic NumPy image so every expected value is derived analytically.
"""

import inspect
import os
import tempfile

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

importErr = False
reasonErr = ""
try:
    import cv2
except ImportError:
    importErr = True
    reasonErr = "OpenCV (cv2) not installed"

try:
    from xmera.fswAlgorithms import fpgaImagePipeline
except ImportError:
    importErr = True
    reasonErr = "fpgaImagePipeline not built — check OpenCV option"

from xmera.architecture import messaging  # registers message payload types in SWIG runtime
from xmera.utilities import SimulationBaseClass, macros


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_sim(module):
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("testProcess")
    proc.addTask("testTask", macros.sec2nano(1.0))
    sim.AddModelToTask("testTask", module)
    return sim


def _make_tiff(arr, path):
    cv2.imwrite(path, arr.astype(np.uint16))


def _run_once(sim):
    sim.InitializeSimulation()
    sim.singleStepProcesses()


def _make_module(width, height, kernel=5, threshold=0, roi_size=64):
    mod = fpgaImagePipeline.FpgaImagePipeline()
    mod.ModelTag = "fpgaTest"
    mod.setImageWidth(width)
    mod.setImageHeight(height)
    mod.setKernelSize(kernel)
    mod.setThreshold(threshold)
    mod.setRoiRegionSize(roi_size)
    return mod


# ---------------------------------------------------------------------------
# Shared synthetic image used by the first two tests
#
#   Image  : 16×8, every pixel = 500 (uint16), kernel=5 (half=2)
#   Blur   : interior pixels (rows 2–5, cols 2–13) receive
#            blur = (500 × 25) >> 1 = 6250
#            border pixels (rows 0–1, 6–7 and cols 0–1, 14–15) receive blur = 0
# ---------------------------------------------------------------------------

W_SMALL, H_SMALL = 16, 8
VAL = 500
INTERIOR_BLUR = (VAL * 5 * 5) >> 1   # 6250 — full 5×5 window, blurShift(5)=1
HALF = 2                               # (kernel-1)//2 for kernel=5


# ---------------------------------------------------------------------------
# Test 0 — Smoke test
# ---------------------------------------------------------------------------

@pytest.mark.skipif(importErr, reason=reasonErr)
def test_smoke_small_image():
    """Small 32x32 synthetic image loaded from file; verify rawImageOutMsg dimensions."""
    W, H = 32, 32
    img = np.full((H, W), 500, dtype=np.uint16)

    with tempfile.NamedTemporaryFile(suffix=".tiff", delete=False) as f:
        img_path = f.name
    try:
        _make_tiff(img, img_path)

        mod = _make_module(W, H)
        mod.setImageFileName(img_path)
        sim = _make_sim(mod)
        _run_once(sim)

        rawMsg = mod.rawImageOutMsg.read()
        assert rawMsg.width == W, f"Expected width={W}, got {rawMsg.width}"
        assert rawMsg.height == H, f"Expected height={H}, got {rawMsg.height}"
        assert rawMsg.imageBufferLength == W * H * 2  # sizeof(uint16_t) = 2
    finally:
        os.unlink(img_path)


# ---------------------------------------------------------------------------
# Test 1 — applyBlurAndThreshold()
# ---------------------------------------------------------------------------

@pytest.mark.skipif(importErr, reason=reasonErr)
@pytest.mark.parametrize("threshold, interior_set", [
    (INTERIOR_BLUR - 1, True),   # 6250 > 6249 → interior bits SET
    (INTERIOR_BLUR,     False),  # 6250 > 6250 is False → all bits CLEAR (strict >)
])
def test_apply_blur_and_threshold(threshold, interior_set):
    """Unit test for applyBlurAndThreshold() — verifies both blurBuf and threshBuf.

    C++ function under test
    -----------------------
    applyBlurAndThreshold() performs two operations in a single pass:
      1. Computes a separable 2-D box blur and writes the result to blurBuf.
      2. Compares each blur output against the threshold and packs the result
         (1 = above threshold, 0 = not) into threshBuf, MSB-first per byte.

    Synthetic image
    ---------------
    16 (W) × 8 (H), every pixel = 500 (uint16).
    Kernel = 5, half = (5-1)/2 = 2.

    Blur formula (NOT a true average)
    ----------------------------------
    The FPGA blur computes:
        blur = sum(k×k window) >> blurShift(k)
    blurShift(5) = 1, so the divisor is 2, not k²=25.

    For a uniform image every pixel in the window = 500, so:
        interior blur = (500 × 5 × 5) >> 1 = 12500 >> 1 = 6250

    This is intentional hardware behaviour: right-shifting by 1 is free on an
    FPGA, whereas dividing by 25 would require an expensive divider circuit.
    The threshold must therefore be calibrated to the blur output scale (~6250),
    not the raw pixel scale (~500).

    Pixel regions
    -------------
    Interior (rows 2–5, cols 2–13):
        The full 5×5 window fits inside the image → blur = 6250.
    Border (rows 0–1, rows 6–7, cols 0–1, cols 14–15):
        The pipeline never writes the border strip → blur = 0.

    Parametrized threshold cases
    ----------------------------
    threshold = 6249:  6250 > 6249 is True  → interior bits SET,  border bits CLEAR.
    threshold = 6250:  6250 > 6250 is False → ALL bits CLEAR.
    The second case specifically verifies the comparison is strict > (not >=).

    Assertions (every pixel checked)
    ---------------------------------
    getBlurPixel(idx) == 6250  for interior pixels,  0 for border pixels.
    getThreshBit(idx) == True  for interior pixels when threshold=6249,
                        False  for all pixels when threshold=6250.
    """
    img = np.full((H_SMALL, W_SMALL), VAL, dtype=np.uint16)
    with tempfile.NamedTemporaryFile(suffix=".tiff", delete=False) as f:
        img_path = f.name
    try:
        _make_tiff(img, img_path)
        mod = _make_module(W_SMALL, H_SMALL, kernel=5, threshold=threshold)
        mod.setImageFileName(img_path)
        _run_once(_make_sim(mod))

        for r in range(H_SMALL):
            for c in range(W_SMALL):
                idx = r * W_SMALL + c
                interior = HALF <= r < H_SMALL - HALF and HALF <= c < W_SMALL - HALF
                expected_blur = INTERIOR_BLUR if interior else 0
                expected_bit  = interior_set  if interior else False

                assert mod.getBlurPixel(idx) == expected_blur, (
                    f"blur  ({r},{c}): expected {expected_blur}, got {mod.getBlurPixel(idx)}"
                )
                assert mod.getThreshBit(idx) == expected_bit, (
                    f"bit   ({r},{c}): expected {expected_bit}, got {mod.getThreshBit(idx)}"
                )
    finally:
        os.unlink(img_path)


# ---------------------------------------------------------------------------
# Test 2 — computeRowColSums()
# ---------------------------------------------------------------------------

@pytest.mark.skipif(importErr, reason=reasonErr)
def test_compute_row_col_sums():
    """Unit test for computeRowColSums() — verifies rowSumBuf and colSumBuf.

    C++ function under test
    -----------------------
    computeRowColSums() reads threshBuf (produced by applyBlurAndThreshold) and
    accumulates two 1-D histograms:
        rowSumBuf[r]  +=1  for each set bit at column c in row r
        colSumBuf[c]  +=1  for each set bit at row r in column c

    It reads only threshBuf — it does not re-read blur values or raw pixels.

    Synthetic image and pipeline settings
    --------------------------------------
    Same 16×8 uniform image (val=500) and kernel=5 as test_apply_blur_and_threshold.
    threshold = INTERIOR_BLUR - 1 = 6249, so ALL interior pixels are above threshold.

    Known threshBuf pattern (trust test_apply_blur_and_threshold)
    --------------------------------------------------------------
    Interior rectangle rows 2–5 × cols 2–13: all bits SET  (48 set bits total).
    Border rows 0–1, 6–7 and cols 0–1, 14–15: all bits CLEAR.

    Expected rowSumBuf  (H=8 entries)
    ------------------------------------
    row 0:    0   — border, no set bits in this row
    row 1:    0   — border
    rows 2–5: 12  — 12 set bits per row (cols 2, 3, …, 13 are all set)
    row 6:    0   — border
    row 7:    0   — border

    Expected colSumBuf  (W=16 entries)
    ------------------------------------
    col 0:      0   — border, no set bits in this column
    col 1:      0   — border
    cols 2–13:  4   — 4 set bits per column (rows 2, 3, 4, 5 are all set)
    col 14:     0   — border
    col 15:     0   — border

    Cross-check: sum(rowSumBuf) = 4×12 = 48 = sum(colSumBuf) = 12×4 = 48.
    """
    THRESHOLD = INTERIOR_BLUR - 1   # 6249 → all interior bits set

    EXPECTED_ROW_SUMS = [0, 0, 12, 12, 12, 12, 0, 0]
    EXPECTED_COL_SUMS = [0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0, 0]

    img = np.full((H_SMALL, W_SMALL), VAL, dtype=np.uint16)
    with tempfile.NamedTemporaryFile(suffix=".tiff", delete=False) as f:
        img_path = f.name
    try:
        _make_tiff(img, img_path)
        mod = _make_module(W_SMALL, H_SMALL, kernel=5, threshold=THRESHOLD)
        mod.setImageFileName(img_path)
        _run_once(_make_sim(mod))

        for r in range(H_SMALL):
            assert mod.getRowSum(r) == EXPECTED_ROW_SUMS[r], (
                f"rowSum[{r}]: expected {EXPECTED_ROW_SUMS[r]}, got {mod.getRowSum(r)}"
            )
        for c in range(W_SMALL):
            assert mod.getColSum(c) == EXPECTED_COL_SUMS[c], (
                f"colSum[{c}]: expected {EXPECTED_COL_SUMS[c]}, got {mod.getColSum(c)}"
            )
    finally:
        os.unlink(img_path)


# ---------------------------------------------------------------------------
# Test 3 — computeRoi()
# ---------------------------------------------------------------------------

@pytest.mark.skipif(importErr, reason=reasonErr)
def test_compute_roi():
    """Unit test for computeRoi() — verifies region ranking by above-threshold count.

    C++ function under test
    -----------------------
    computeRoi() divides the image into a grid of roi_size×roi_size regions,
    counts the above-threshold pixels (from threshBuf) in each region, then
    partial-sorts to find the top FPGA_ROI_TOP_COUNT (=8) regions by count and
    writes them in descending order into FpgaBinsMsgPayload.topBins[].

    Synthetic image
    ---------------
    192×192, all pixels = 0 except a 64×64 bright block (value 4095) at
    raw rows [64..127], cols [64..127].

    Pipeline settings
    -----------------
    kernel=5 (half=2), threshold=100, roi_size=64.
    The image forms a 3×3 grid of 9 regions (192/64 = 3 per side):
        (row=0,col=0)  (row=0,col=1)  (row=0,col=2)
        (row=1,col=0)  (row=1,col=1)  (row=1,col=2)   ← bright block here
        (row=2,col=0)  (row=2,col=1)  (row=2,col=2)

    Above-threshold pixel derivation
    ---------------------------------
    The 5×5 blur window at (rStart, c) overlaps the bright block [64..127]×[64..127]
    when rStart ∈ [60..127] and c ∈ [60..127], giving:
        outRow = rStart+2  ∈ [62..129]
        outCol = c+2       ∈ [62..129]
    Any overlap — even 1 bright pixel — produces blur ≥ 4095>>1 = 2047 >> threshold=100,
    so all 68×68 = 4624 positions in [62..129]×[62..129] are above threshold.

    Count per region (roi_size=64, regions cover pixel rows/cols [0..63], [64..127], [128..191])
    --------------------------------------------------------------------------------------------
    Region (0,0): outRows [62,63] × outCols [62,63]   =  2× 2 =    4
    Region (0,1): outRows [62,63] × outCols [64,127]  =  2×64 =  128
    Region (0,2): outRows [62,63] × outCols [128,129] =  2× 2 =    4
    Region (1,0): outRows [64,127] × outCols [62,63]  = 64× 2 =  128
    Region (1,1): outRows [64,127] × outCols [64,127] = 64×64 = 4096  ← ranked #1
    Region (1,2): outRows [64,127] × outCols [128,129]= 64× 2 =  128
    Region (2,0): outRows [128,129] × outCols [62,63] =  2× 2 =    4
    Region (2,1): outRows [128,129] × outCols [64,127]=  2×64 =  128
    Region (2,2): outRows [128,129] × outCols [128,129]=  2× 2 =    4
    Total = 4624.

    numValidRegions
    ---------------
    All 9 regions have nonzero counts, but FPGA_ROI_TOP_COUNT = 8, so
    numValidRegions = min(9, 8) = 8.

    Recorder access pattern
    -----------------------
    roiOutMsg is accessed via a Basilisk recorder (not .read()) because
    FpgaBinsMsgPayload contains a struct array (topBins[8]) that requires
    STRUCTASLIST SWIG typemaps — these are only active in the recorder's
    message module.  The recorder exposes topBins as a dict with keys
    "topBins[k].row", "topBins[k].col", "topBins[k].count".

    Expected roiOutMsg (recorder access)
    --------------------------------------
    numValidRegions    = 8
    topBins[0].row   = 1     (region row index in the 3×3 grid)
    topBins[0].col   = 1     (region col index in the 3×3 grid)
    topBins[0].count = 4096  (number of above-threshold pixels in region (1,1))
    """
    ROI_SIZE = 64
    W, H = 3 * ROI_SIZE, 3 * ROI_SIZE   # 192×192

    img = np.zeros((H, W), dtype=np.uint16)
    img[ROI_SIZE:2 * ROI_SIZE, ROI_SIZE:2 * ROI_SIZE] = 4095   # center region (1,1)

    with tempfile.NamedTemporaryFile(suffix=".tiff", delete=False) as f:
        img_path = f.name
    try:
        _make_tiff(img, img_path)
        mod = _make_module(W, H, kernel=5, threshold=100, roi_size=ROI_SIZE)
        mod.setImageFileName(img_path)
        sim = _make_sim(mod)
        roi_rec = mod.roiOutMsg.recorder()
        sim.AddModelToTask("testTask", roi_rec)
        _run_once(sim)

        num_valid = roi_rec.numValidRegions[0]
        top       = roi_rec.topBins[0]
        assert num_valid == 8, (
            f"numValidRegions: expected 8, got {num_valid}"
        )
        assert int(top["topBins[0].row"])   == 1,    f"top.row:   expected 1,    got {top['topBins[0].row']}"
        assert int(top["topBins[0].col"])   == 1,    f"top.col:   expected 1,    got {top['topBins[0].col']}"
        assert int(top["topBins[0].count"]) == 4096, f"top.count: expected 4096, got {top['topBins[0].count']}"
    finally:
        os.unlink(img_path)


# ---------------------------------------------------------------------------
# Test 4 - Blur test for single impulse
# ---------------------------------------------------------------------------
@pytest.mark.skipif(importErr, reason=reasonErr)
def test_blur_impulse_kernel5():
    """Unit test for applyBlurAndThreshold() — verifies blur spatial support using a
    single-pixel impulse image.

    C++ function under test
    -----------------------
    applyBlurAndThreshold() implements a separable 2-D box blur:
        Row pass  : for each pixel (r, c), sum the row window [c-half .. c+half].
        Column pass: for each result (r, c), sum the column window [r-half .. r+half].
        Final blur = (row_sum × col_sum product) >> blurShift(k)
    For kernel=5: half=2, blurShift=1, so the 2-D sum is right-shifted by 1.

    Synthetic image
    ---------------
    16×16, all pixels = 0 except a single bright pixel at (row=7, col=7) = 4095.
    This is an impulse (delta function), which isolates the blur kernel's spatial
    support: the blur output is nonzero only where the sliding window overlaps the
    impulse location.

    Expected nonzero region
    -----------------------
    The 5×5 window (half=2) centred on output pixel (r, c) overlaps the impulse at
    (cy=7, cx=7) only when:
        |r - 7| <= 2  AND  |c - 7| <= 2
    That is a 5×5 block: rows 5–9, cols 5–9.

    For any (r, c) in that block, exactly one pixel in the window is nonzero (4095),
    so the full-window sum = 4095, and:
        blur = 4095 >> 1 = 2047

    At the centre (r=7, c=7) the full 5×5 window is used, giving the same 2047.

    Two assertions
    --------------
    1. Peak check: getBlurPixel(7*16+7) == 2047.
       Only one pixel in the 5×5 window is nonzero (4095), so
       colSum = 4095 and blur = 4095 >> 1 = 2047.

    2. Support check: for all pixels with |r-7| > half OR |c-7| > half,
       getBlurPixel == 0.
       The condition |r-cy| > half OR |c-cx| > half exactly identifies
       pixels outside the 5×5 nonzero region — any blur energy leaking
       beyond that boundary triggers a failure.
    """
    W, H = 16, 16
    img = np.zeros((H, W), dtype=np.uint16)
    cx, cy = 7, 7  # center of impulse (col, row)
    img[cy, cx] = 4095

    with tempfile.NamedTemporaryFile(suffix=".tiff", delete=False) as f:
        img_path = f.name
    try:
        _make_tiff(img, img_path)
        mod = _make_module(W, H, kernel=5, threshold=0)
        mod.setImageFileName(img_path)
        sim = _make_sim(mod)
        _run_once(sim)

        half = 2
        # The full 5×5 window is centred on (cy,cx): sum = 4095 (one nonzero pixel),
        # blur = 4095 >> 1 = 2047.
        peak = mod.getBlurPixel(cy * W + cx)
        assert peak == 2047, f"Peak of impulse response: expected 2047, got {peak}"

        # All pixels outside the 5×5 support (|r-cy|>half OR |c-cx|>half) must be zero.
        for r in range(H):
            for c in range(W):
                v = mod.getBlurPixel(r * W + c)
                if abs(r - cy) > half or abs(c - cx) > half:
                    assert v == 0, f"Expected zero at ({r},{c}) outside 5×5 support, got {v}"
    finally:
        os.unlink(img_path)


# ---------------------------------------------------------------------------
# Test 5 - Message chaining
# ---------------------------------------------------------------------------

@pytest.mark.skipif(importErr, reason=reasonErr)
def test_message_chaining_recorder():
    """rawImageOutMsg fields are correctly written and readable downstream."""
    W, H = 16, 16
    img = np.full((H, W), 1000, dtype=np.uint16)

    with tempfile.NamedTemporaryFile(suffix=".tiff", delete=False) as f:
        img_path = f.name
    try:
        _make_tiff(img, img_path)
        mod = _make_module(W, H)
        mod.setImageFileName(img_path)

        sim = _make_sim(mod)

        # Add a recorder for rawImageOutMsg
        rawRec = mod.rawImageOutMsg.recorder()
        sim.AddModelToTask("testTask", rawRec)

        _run_once(sim)

        # Verify width and height were recorded correctly
        assert rawRec.width[-1] == W, f"Recorded width {rawRec.width[-1]} != {W}"
        assert rawRec.height[-1] == H, f"Recorded height {rawRec.height[-1]} != {H}"
        assert rawRec.imageBufferLength[-1] == W * H * 2
    finally:
        os.unlink(img_path)
