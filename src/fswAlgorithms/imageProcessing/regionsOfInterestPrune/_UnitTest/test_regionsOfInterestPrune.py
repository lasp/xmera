"""
Unit tests for regionsOfInterestPrune.

    test_message_connection  — verifies rowColSumInMsg links to fpgaImagePipeline.rowColSumOutMsg
                               and that regionsIdentifiedOutMsg is written after one step.
    test_step2_*             — verify bounding-box identification and the center-coordinate
                               conversion published to regionsIdentifiedOutMsg.
    test_pruning             — end-to-end integration on a real image; writes an annotated
                               PNG using center coordinates (matching saveVisualization).
"""

import inspect
import os
import tempfile
import types

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

IMAGE_PATH = os.path.join(path, "pia_958_830.tiff")

importErr = False
reasonErr = ""
try:
    import cv2
    from xmera.fswAlgorithms import fpgaImagePipeline
    from xmera.fswAlgorithms import regionsOfInterestPrune
except ImportError as e:
    importErr = True
    reasonErr = f"Required module not built: {e}"

# Synthetic image parameters (uniform 500, kernel=5)
# Interior blur = (500 × 5 × 5) >> 1 = 6250; blurShift(5) = 1
# Interior region starts at HALF = (5-1)//2 = 2
HALF = 2
INTERIOR_BLUR = (500 * 5 * 5) >> 1  # 6250

from xmera.architecture import messaging
MAX_NUMBER_REGIONS = messaging.MAX_NUMBER_REGIONS
from xmera.utilities import SimulationBaseClass, macros


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_sim(*modules):
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("testProcess")
    proc.addTask("testTask", macros.sec2nano(1.0))
    for mod in modules:
        sim.AddModelToTask("testTask", mod)
    return sim


def _run_once(sim):
    sim.InitializeSimulation()
    sim.TotalSim.singleStepProcesses()


def _regions_from_log(roi_log, timestep=-1):
    """Return valid regions at timestep as SimpleNamespace objects with field attribute access."""
    regions = []
    for k in range(MAX_NUMBER_REGIONS):
        numberOfPixels = roi_log.numberOfPixels[timestep][k]
        if numberOfPixels > 0:
            regions.append(types.SimpleNamespace(
                numberOfPixels=numberOfPixels,
                centerX=roi_log.centerX[timestep][k],
                centerY=roi_log.centerY[timestep][k],
                width=roi_log.width[timestep][k],
                height=roi_log.height[timestep][k],
            ))
    return regions


# ---------------------------------------------------------------------------
# Test — message connection
# ---------------------------------------------------------------------------

@pytest.mark.skipif(importErr, reason=reasonErr)
def test_message_connection():
    """Verify rowColSumInMsg connects to fpgaImagePipeline.rowColSumOutMsg and
    that regionsIdentifiedOutMsg is written with correct image dimensions.

    Assertions
    ----------
    1. rowColSumInMsg.isLinked() is True after subscribeTo.
    2. rowColSumOutMsg carries the correct image dimensions after one step.
    3. regionsIdentifiedOutMsg is written; any valid region has centre coordinates
       within the image bounds.
    """
    assert os.path.isfile(IMAGE_PATH), f"Test image not found: {IMAGE_PATH}"

    img_info = cv2.imread(IMAGE_PATH, cv2.IMREAD_ANYDEPTH | cv2.IMREAD_GRAYSCALE)
    assert img_info is not None, f"cv2 could not read: {IMAGE_PATH}"
    img_h, img_w = img_info.shape

    pipeline = fpgaImagePipeline.FpgaImagePipeline()
    pipeline.ModelTag = "fpgaPipeline"
    pipeline.setImageWidth(img_w)
    pipeline.setImageHeight(img_h)
    pipeline.setImageFileName(IMAGE_PATH)

    pruner = regionsOfInterestPrune.RegionsOfInterestPrune()
    pruner.ModelTag = "roiPrune"
    pruner.rowColSumInMsg.subscribeTo(pipeline.rowColSumOutMsg)

    assert pruner.rowColSumInMsg.isLinked(), \
        "rowColSumInMsg should be linked after subscribeTo()"

    rc_log  = pipeline.rowColSumOutMsg.recorder()
    roi_log = pruner.regionsIdentifiedOutMsg.recorder()
    _run_once(_make_sim(pipeline, pruner, rc_log, roi_log))

    assert rc_log.numRows[-1] == img_h, \
        f"Expected numRows={img_h}, got {rc_log.numRows[-1]}"
    assert rc_log.numCols[-1] == img_w, \
        f"Expected numCols={img_w}, got {rc_log.numCols[-1]}"

    # regionsIdentifiedOutMsg must be written; any valid region must be in-bounds
    for reg in _regions_from_log(roi_log):
        assert 0 <= reg.centerX < img_w, \
            f"centerX={reg.centerX} out of bounds [0, {img_w})"
        assert 0 <= reg.centerY < img_h, \
            f"centerY={reg.centerY} out of bounds [0, {img_h})"


# ---------------------------------------------------------------------------
# Tests — rank-1/rank-2 identification
# ---------------------------------------------------------------------------
#
# Synthetic image building blocks: bright blocks of pixel value 500 separated
# by at least one dark pixel so each block produces a distinct row/col span.
#
# A 5×5 block at value 500 with kernel=5:
#   interior blur = (500×5×5)>>1 = 6250 > threshold=6249  → 1 above-threshold pixel
#   border pixels: ≤4 full rows/cols in kernel → blur ≤ 5000 < 6249
#
# Internal (corner-based) representation: row, col, height, width, count
# Published output (RegionsIdentifiedMsgPayload): centerX, centerY, width, height,
#   numberOfPixels  where centerX = col + width//2, centerY = row + height//2.

BLOCK  = 5          # 5×5 bright block
GAP    = 1          # dark gap between blocks
STRIDE = BLOCK + GAP

LARGE_BLOCK  = 13   # 13×13 block → 9×9 = 81 interior above-threshold pixels
MEDIUM_BLOCK = 9    # 9×9  block → 5×5 = 25 interior above-threshold pixels


def _write_tmp_image(img):
    with tempfile.NamedTemporaryFile(suffix=".tiff", delete=False) as f:
        path = f.name
    cv2.imwrite(path, img)
    return path


@pytest.mark.skipif(importErr, reason=reasonErr)
def test_step2_rank1_only():
    """M=1: single object — one above-threshold pixel at the block interior.

    Published output:  regions[0].numberOfPixels==1, center at (col+0, row+0)
                       since width==height==1.
    """
    H = 2 * HALF + STRIDE
    W = 2 * HALF + STRIDE
    img = np.zeros((H, W), dtype=np.uint16)
    img[HALF:HALF + BLOCK, HALF:HALF + BLOCK] = 500

    # Single above-threshold pixel: interior corner (row=HALF+HALF, col=HALF+HALF) = (4, 4)
    exp_col, exp_row, exp_w, exp_h = HALF + HALF, HALF + HALF, 1, 1
    exp_cx = exp_col + exp_w // 2   # = 4
    exp_cy = exp_row + exp_h // 2   # = 4

    img_path = _write_tmp_image(img)
    try:
        pipeline = fpgaImagePipeline.FpgaImagePipeline()
        pipeline.ModelTag = "fpga"
        pipeline.setImageWidth(W)
        pipeline.setImageHeight(H)
        pipeline.setImageFileName(img_path)
        pipeline.setKernelSize(5)
        pipeline.setThreshold(INTERIOR_BLUR - 1)

        pruner = regionsOfInterestPrune.RegionsOfInterestPrune()
        pruner.ModelTag = "pruner"
        pruner.rowColSumInMsg.subscribeTo(pipeline.rowColSumOutMsg)

        roi_log = pruner.regionsIdentifiedOutMsg.recorder()
        _run_once(_make_sim(pipeline, pruner, roi_log))

        # --- published message checks (center coordinates) ---
        regions = _regions_from_log(roi_log)
        assert len(regions) == 1, \
            f"Expected 1 valid region in regionsIdentifiedOutMsg, got {len(regions)}"
        r = regions[0]
        assert r.numberOfPixels == 1, \
            f"Expected numberOfPixels=1, got {r.numberOfPixels}"
        assert r.centerX == exp_cx and r.centerY == exp_cy, \
            f"Expected center=({exp_cx},{exp_cy}), got ({r.centerX},{r.centerY})"
        assert r.width == exp_w and r.height == exp_h, \
            f"Expected size={exp_w}×{exp_h}, got {r.width}×{r.height}"
    finally:
        os.unlink(img_path)


@pytest.mark.skipif(importErr, reason=reasonErr)
def test_step2_rank1_and_rank2():
    """Two real objects; both appear in regionsIdentifiedOutMsg with correct
    center coordinates.

    Image layout (160×160):
      13×13 block at (3, 3)  → rank-1; bounding box row=5,col=5,h=9,w=9
                               → centerX=9, centerY=9
       5×5 block at (100, 5) → rank-2; bounding box row=102,col=5,h=1,w=9
                               → centerX=9, centerY=102
    """
    H, W = 160, 160
    img = np.zeros((H, W), dtype=np.uint16)
    img[3:3 + LARGE_BLOCK, 3:3 + LARGE_BLOCK] = 500   # large block → rank-1
    img[100:100 + BLOCK, 5:5 + BLOCK] = 500            # small block → rank-2

    img_path = _write_tmp_image(img)
    try:
        pipeline = fpgaImagePipeline.FpgaImagePipeline()
        pipeline.ModelTag = "fpga"
        pipeline.setImageWidth(W)
        pipeline.setImageHeight(H)
        pipeline.setImageFileName(img_path)
        pipeline.setKernelSize(5)
        pipeline.setThreshold(INTERIOR_BLUR - 1)

        pruner = regionsOfInterestPrune.RegionsOfInterestPrune()
        pruner.ModelTag = "pruner"
        pruner.rowColSumInMsg.subscribeTo(pipeline.rowColSumOutMsg)

        roi_log = pruner.regionsIdentifiedOutMsg.recorder()
        _run_once(_make_sim(pipeline, pruner, roi_log))

        # --- published message checks (center coordinates) ---
        regions = _regions_from_log(roi_log)
        assert len(regions) == 2, \
            f"Expected 2 valid regions in regionsIdentifiedOutMsg, got {len(regions)}"

        r1 = regions[0]
        assert r1.centerX == 9 and r1.centerY == 9, \
            f"rank-1 center: expected (9,9), got ({r1.centerX},{r1.centerY})"
        assert r1.width == 9 and r1.height == 9, \
            f"rank-1 size: expected 9×9, got {r1.width}×{r1.height}"
        r2 = regions[1]
        assert r2.centerX == 9 and r2.centerY == 102, \
            f"rank-2 center: expected (9,102), got ({r2.centerX},{r2.centerY})"
        assert r2.width == 9 and r2.height == 1, \
            f"rank-2 size: expected 9×1, got {r2.width}×{r2.height}"
    finally:
        os.unlink(img_path)


@pytest.mark.skipif(importErr, reason=reasonErr)
def test_step2_rank2_by_count():
    """Three objects sorted by pixel count; all three fit in MAX_NUMBER_REGIONS=3
    and appear in regionsIdentifiedOutMsg in the correct order.

    Image layout (H=100, W=30):
      13×13 block at (3, 3)   → rank-1; bbox row=5,col=5,h=9,w=9   → center (9,9)
       9×9  block at (50, 5)  → rank-2; bbox row=52,col=5,h=5,w=9  → center (9,54)
       5×5  block at (80, 7)  → rank-3; bbox row=82,col=5,h=1,w=9  → center (9,82)
    """
    H, W = 100, 30
    img = np.zeros((H, W), dtype=np.uint16)
    img[3:3 + LARGE_BLOCK,   3:3 + LARGE_BLOCK]   = 500   # 13×13 → rank-1
    img[50:50 + MEDIUM_BLOCK, 5:5 + MEDIUM_BLOCK]  = 500   # 9×9  → rank-2
    img[80:80 + BLOCK,        7:7 + BLOCK]          = 500   # 5×5  → rank-3

    img_path = _write_tmp_image(img)
    try:
        pipeline = fpgaImagePipeline.FpgaImagePipeline()
        pipeline.ModelTag = "fpga"
        pipeline.setImageWidth(W)
        pipeline.setImageHeight(H)
        pipeline.setImageFileName(img_path)
        pipeline.setKernelSize(5)
        pipeline.setThreshold(INTERIOR_BLUR - 1)

        pruner = regionsOfInterestPrune.RegionsOfInterestPrune()
        pruner.ModelTag = "pruner"
        pruner.rowColSumInMsg.subscribeTo(pipeline.rowColSumOutMsg)

        roi_log = pruner.regionsIdentifiedOutMsg.recorder()
        _run_once(_make_sim(pipeline, pruner, roi_log))

        # --- published message checks (center coordinates, all 3 fit in MAX_NUMBER_REGIONS) ---
        regions = _regions_from_log(roi_log)
        assert len(regions) == 3, \
            f"Expected 3 valid regions in regionsIdentifiedOutMsg, got {len(regions)}"

        expected_centers = [(9, 9), (9, 54), (9, 82)]
        for k, (exp_cx, exp_cy) in enumerate(expected_centers):
            r = regions[k]
            assert r.centerX == exp_cx and r.centerY == exp_cy, \
                f"rank-{k+1} center: expected ({exp_cx},{exp_cy}), got ({r.centerX},{r.centerY})"
        # Verify descending order by pixel count
        counts = [r.numberOfPixels for r in regions]
        assert counts == sorted(counts, reverse=True), \
            f"Regions not sorted by numberOfPixels (descending): {counts}"
    finally:
        os.unlink(img_path)


# ---------------------------------------------------------------------------
# Test — end-to-end pipeline on real image
# ---------------------------------------------------------------------------

PIPELINE_KERNEL = 5


def _auto_threshold(image_path, kernel_size, percentile=95.0):
    """Detect image dimensions and compute a threshold from the image's blur distribution.

    Mirrors _detect_image_info() in run_pipeline_reference.py exactly:
    applies the same ×16 scaling for 8-bit images, same blurShift lookup,
    and same percentile default (95.0) so thresholds are consistent across
    both tools regardless of which pia_ image is used.

    Returns
    -------
    (width : int, height : int, threshold : int)
    """
    img = cv2.imread(image_path, cv2.IMREAD_ANYDEPTH | cv2.IMREAD_GRAYSCALE)
    assert img is not None, f"cv2 could not read: {image_path}"
    h, w = img.shape
    pixels = img.astype(np.float32) * (16.0 if img.dtype == np.uint8 else 1.0)
    blur = cv2.boxFilter(pixels, -1, (kernel_size, kernel_size), normalize=False)
    shift = {5: 1, 7: 2, 9: 3}.get(int(kernel_size), 1)
    blur_shifted = blur / float(1 << shift)
    return w, h, max(1, int(np.percentile(blur_shifted.ravel(), percentile)))


@pytest.mark.skipif(importErr, reason=reasonErr)
@pytest.mark.parametrize("test_image_path", [
    os.path.join(path, "pia_958_830.tiff"),
    # Additional images can be exercised by copying them into this test directory:
    # os.path.join(path, "pia_616_592.tiff"),
    # os.path.join(path, "bennu_1024_1024.tiff"),
    # os.path.join(path, "jupiter_3000_3000.tiff"),
])
@pytest.mark.parametrize("row_col_span", [2, 3, 4])
def test_pruning(test_image_path, row_col_span, tmp_path, threshold=None):
    """End-to-end integration test: real pia_ image through the full
    fpgaImagePipeline → regionsOfInterestPrune chain.

    Checks at least one candidate is identified and verifies the published
    regionsIdentifiedOutMsg contains valid center-coordinate regions.

    The module writes a diagnostic PNG ("<timeTag>_pruning_output.png") to the
    per-test temp directory: all published regions (up to MAX_NUMBER_REGIONS)
    drawn in thin yellow from centre coordinates, matching the production
    saveVisualization style. Rank-1 in RED with filled centre dot and label
    "R1 (<numberOfPixels>)"; Rank-2 in BLUE with filled centre dot (if present).
    """
    if not os.path.isfile(test_image_path):
        pytest.skip(f"Test image not found: {test_image_path}")

    img_w, img_h, auto_t = _auto_threshold(test_image_path, PIPELINE_KERNEL)
    if threshold is None:
        threshold = auto_t
    print(f"\ntest_pruning: {img_w}×{img_h}, threshold={threshold}")

    pipeline = fpgaImagePipeline.FpgaImagePipeline()
    pipeline.ModelTag = "fpga"
    pipeline.setImageWidth(img_w)
    pipeline.setImageHeight(img_h)
    pipeline.setImageFileName(test_image_path)
    pipeline.setKernelSize(PIPELINE_KERNEL)
    pipeline.setThreshold(threshold)

    pruner = regionsOfInterestPrune.RegionsOfInterestPrune()
    pruner.ModelTag = "pruner"
    pruner.rowColSumInMsg.subscribeTo(pipeline.rowColSumOutMsg)
    pruner.threshImageInMsg.subscribeTo(pipeline.threshImageOutMsg)
    pruner.setMaxRowSpans(row_col_span)
    pruner.setMaxColSpans(row_col_span)
    pruner.setSaveImages(True)
    # Write diagnostic images to a unique per-test temp dir
    save_dir = str(tmp_path)
    pruner.setSaveDir(save_dir)

    roi_log = pruner.regionsIdentifiedOutMsg.recorder()
    _run_once(_make_sim(pipeline, pruner, roi_log))

    regions = _regions_from_log(roi_log)
    assert len(regions) >= 1, \
        f"Expected at least 1 candidate, got {len(regions)}"

    for k, reg in enumerate(regions):
        assert 0 <= reg.centerX < img_w and 0 <= reg.centerY < img_h, \
            f"Region {k} center ({reg.centerX},{reg.centerY}) out of image bounds {img_w}×{img_h}"

    r1 = regions[0]
    count_str = f"R1 pixel count (approx): {r1.numberOfPixels}"
    if len(regions) >= 2:
        count_str += f"  |  R2 pixel count (approx): {regions[1].numberOfPixels}"
    print(count_str)
    print(f"test_pruning: {len(regions)} candidate(s); visualisation saved by module to {save_dir}")
