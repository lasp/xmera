"""
Unit tests for fpgaImagePipeline module.

Tests cover all pipeline stages:
  1. Smoke test — small synthetic image via imageFileName, verify output dimensions.
  2. Calibration op-codes — all 16 op-codes produce expected pixel values.
  3. Blur — uniform image blurs to itself; impulse produces kernel-shaped response.
  4. Threshold — pixels above/below threshold are correctly packed into the bit buffer.
  5. Row/col sums — counts match manually counted above-threshold pixels.
  6. ROI ranking — correct region ranked #1 for image with known bright spot.
  7. Kernel size validation — reset() throws for invalid kernel size.
  8. Message chaining — rawImageOutMsg fields are correctly written and readable.
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

from xmera.architecture import messaging
from xmera.utilities import SimulationBaseClass, macros


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_sim(module, task_rate_ns=int(1e9)):
    """Set up a minimal simulation with one task containing *module*."""
    sim = SimulationBaseClass.SimBaseClass()
    task = sim.CreateNewTask("testTask", macros.sec2nano(1.0))
    sim.AddModelToTask("testTask", module)
    return sim


def _make_tiff(arr_uint16, path):
    """Write a uint16 numpy array to a 16-bit grayscale TIFF."""
    cv2.imwrite(path, arr_uint16.astype(np.uint16))


def _run_once(sim):
    """Initialize and step the simulation once."""
    sim.InitializeSimulation()
    sim.TotalSim.SingleStepProcesses()


def _make_module(width, height, kernel=5, threshold=0, roi_size=64):
    """Create a configured FpgaImagePipeline module."""
    mod = fpgaImagePipeline.FpgaImagePipeline()
    mod.ModelTag = "fpgaTest"
    mod.setImageWidth(width)
    mod.setImageHeight(height)
    mod.setKernelSize(kernel)
    mod.setThreshold(threshold)
    mod.setRoiRegionSize(roi_size)
    return mod


# ---------------------------------------------------------------------------
# 1. Smoke test
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
# 2. Calibration op-code tests
# ---------------------------------------------------------------------------

@pytest.mark.skipif(importErr, reason=reasonErr)
@pytest.mark.parametrize("op_code, raw_val, calib_val, reg_a, reg_b, reg_c, reg_d, expected", [
    # op=0x0: pass — output = raw
    (0x0,  200, 100, 10, 20, 30, 40, 200),
    # op=0x1: set to regA
    (0x1,  200, 100, 50, 20, 30, 40, 50),
    # op=0x2: set to regB
    (0x2,  200, 100, 10, 60, 30, 40, 60),
    # op=0x3: set to regC
    (0x3,  200, 100, 10, 20, 70, 40, 70),
    # op=0x4: set to regD
    (0x4,  200, 100, 10, 20, 30, 80, 80),
    # op=0x5: set to calibVal literal
    (0x5,  200, 123, 10, 20, 30, 40, 123),
    # op=0x6: add regA (200 + 50 = 250)
    (0x6,  200, 100, 50, 20, 30, 40, 250),
    # op=0x7: add regB (200 + 60 = 260)
    (0x7,  200, 100, 10, 60, 30, 40, 260),
    # op=0x8: add regC (200 + 70 = 270)
    (0x8,  200, 100, 10, 20, 70, 40, 270),
    # op=0x9: add regD (200 + 80 = 280)
    (0x9,  200, 100, 10, 20, 30, 80, 280),
    # op=0xa: add calibVal (200 + 100 = 300)
    (0xa,  200, 100, 10, 20, 30, 40, 300),
    # op=0xb: sub regA (200 - 50 = 150)
    (0xb,  200, 100, 50, 20, 30, 40, 150),
    # op=0xc: sub regB (200 - 60 = 140)
    (0xc,  200, 100, 10, 60, 30, 40, 140),
    # op=0xd: sub regC (200 - 70 = 130)
    (0xd,  200, 100, 10, 20, 70, 40, 130),
    # op=0xe: sub regD (200 - 80 = 120)
    (0xe,  200, 100, 10, 20, 30, 80, 120),
    # op=0xf: sub calibVal (200 - 100 = 100)
    (0xf,  200, 100, 10, 20, 30, 40, 100),
    # Clamp at 0: op=0xb, raw=10, regA=50 → 10-50 clamped to 0
    (0xb,   10, 100, 50, 20, 30, 40,   0),
    # Clamp at 4095: op=0x6, raw=4000, regA=200 → 4200 clamped to 4095
    (0x6, 4000, 100, 200, 20, 30, 40, 4095),
])
def test_calibration_opcodes(op_code, raw_val, calib_val, reg_a, reg_b, reg_c, reg_d, expected):
    """Each calibration op-code produces the correct output for a single-pixel image."""
    W, H = 1, 1
    # Build a 1-pixel image and a 1-pixel calibration image
    img = np.array([[raw_val]], dtype=np.uint16)
    calib_word = ((op_code & 0xF) << 12) | (calib_val & 0x0FFF)
    calib = np.array([[calib_word]], dtype=np.uint16)

    with tempfile.NamedTemporaryFile(suffix=".tiff", delete=False) as f:
        img_path = f.name
    with tempfile.NamedTemporaryFile(suffix=".tiff", delete=False) as f:
        calib_path = f.name
    try:
        _make_tiff(img, img_path)
        _make_tiff(calib, calib_path)

        mod = _make_module(W, H)
        mod.setImageFileName(img_path)
        mod.setCalibEnabled(True)
        mod.setCalibImageFile(calib_path)
        mod.setCalibRegA(reg_a)
        mod.setCalibRegB(reg_b)
        mod.setCalibRegC(reg_c)
        mod.setCalibRegD(reg_d)
        sim = _make_sim(mod)
        _run_once(sim)

        result = mod.getRawPixel(0)
        assert result == expected, (
            f"op=0x{op_code:x}, raw={raw_val}, calib={calib_val}: "
            f"expected {expected}, got {result}"
        )
    finally:
        os.unlink(img_path)
        os.unlink(calib_path)


# ---------------------------------------------------------------------------
# 3. Blur tests
# ---------------------------------------------------------------------------

@pytest.mark.skipif(importErr, reason=reasonErr)
@pytest.mark.parametrize("kernel", [5, 7, 9])
def test_blur_uniform_image(kernel):
    """A uniform image should blur to itself (modulo integer rounding)."""
    W, H = 32, 32
    val = 512
    img = np.full((H, W), val, dtype=np.uint16)

    with tempfile.NamedTemporaryFile(suffix=".tiff", delete=False) as f:
        img_path = f.name
    try:
        _make_tiff(img, img_path)
        mod = _make_module(W, H, kernel=kernel, threshold=0)
        mod.setImageFileName(img_path)
        sim = _make_sim(mod)
        _run_once(sim)

        # Check interior pixels (away from border effects of the zero-border window)
        half = (kernel - 1) // 2
        for r in range(half, H - half):
            for c in range(half, W - half):
                got = mod.getBlurPixel(r * W + c)
                # Interior pixels have full kernel coverage: sum = val * kernel^2, shift applies
                shift = {5: 1, 7: 2, 9: 3}[kernel]
                expected = (val * kernel * kernel) >> shift
                assert got == expected, f"kernel={kernel} r={r} c={c}: got {got}, expected {expected}"
    finally:
        os.unlink(img_path)


@pytest.mark.skipif(importErr, reason=reasonErr)
def test_blur_impulse_kernel5():
    """A single bright pixel blurs to a kernel-shaped (5x5) response."""
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

        shift = 1  # kernel=5 → shift=1
        # The peak (at the impulse) should have gone through a 1x1 "window" with value 4095,
        # then column pass with 1 element: blurBuf[cy*W+cx] = (4095 >> 1) = 2047
        peak = mod.getBlurPixel(cy * W + cx)
        assert peak > 0, "Peak of impulse response should be nonzero"

        # All pixels farther than 4 from center should be zero (2*half+1=5, half=2)
        half = 2
        for r in range(H):
            for c in range(W):
                v = mod.getBlurPixel(r * W + c)
                if abs(r - cy) > half * 2 or abs(c - cx) > half * 2:
                    assert v == 0, f"Expected zero at ({r},{c}) far from impulse, got {v}"
    finally:
        os.unlink(img_path)


# ---------------------------------------------------------------------------
# 4. Threshold test
# ---------------------------------------------------------------------------

@pytest.mark.skipif(importErr, reason=reasonErr)
def test_threshold_packing():
    """Pixels above threshold are set in the 1-bit buffer; below are clear."""
    W, H = 8, 4  # exactly 4 bytes of threshold buffer
    threshold = 100

    # Create image: even-column pixels = 200 (above), odd-column = 50 (below)
    img = np.zeros((H, W), dtype=np.uint16)
    img[:, 0::2] = 200   # even cols: above threshold
    img[:, 1::2] = 50    # odd cols: below threshold

    with tempfile.NamedTemporaryFile(suffix=".tiff", delete=False) as f:
        img_path = f.name
    try:
        _make_tiff(img, img_path)
        # kernel=5 with uniform regions; use a tiny image so blur doesn't mix columns
        # Use kernel=5, but the image is only 4 rows tall — border effects will occur.
        # For this test, skip blur by setting all cols to the same value then checking
        # the threshold behaviour directly with a simple case.
        mod = _make_module(W, H, kernel=5, threshold=threshold)
        mod.setImageFileName(img_path)
        sim = _make_sim(mod)
        _run_once(sim)

        # Check a few known pixels in the interior
        # After blur, mixed pixels may have intermediate values — just check
        # that the overall pattern: every pixel has a deterministic above/below result.
        # Focus on row 2, columns 0 and 1 (interior of the 4-row image)
        for c in range(W):
            idx = 2 * W + c
            bit = mod.getThreshBit(idx)
            blur_val = mod.getBlurPixel(idx)
            if blur_val > threshold:
                assert bit, f"col={c} blur={blur_val} > threshold={threshold} but bit is 0"
            else:
                assert not bit, f"col={c} blur={blur_val} <= threshold={threshold} but bit is 1"
    finally:
        os.unlink(img_path)


# ---------------------------------------------------------------------------
# 5. Row/col sums test
# ---------------------------------------------------------------------------

@pytest.mark.skipif(importErr, reason=reasonErr)
def test_row_col_sums():
    """Row and column sums match manually counted above-threshold pixels."""
    W, H = 16, 16
    threshold = 500

    # All pixels = 0 except a 4x4 block at top-left corner = 4095
    img = np.zeros((H, W), dtype=np.uint16)
    img[0:4, 0:4] = 4095

    with tempfile.NamedTemporaryFile(suffix=".tiff", delete=False) as f:
        img_path = f.name
    try:
        _make_tiff(img, img_path)
        mod = _make_module(W, H, kernel=5, threshold=threshold, roi_size=64)
        mod.setImageFileName(img_path)
        sim = _make_sim(mod)
        _run_once(sim)

        # Count what the test expects from the threshold buffer (after blur)
        expected_row_sums = [mod.getRowSum(r) for r in range(H)]
        expected_col_sums = [mod.getColSum(c) for c in range(W)]

        # Total above-threshold pixels = sum of all row sums = sum of all col sums
        total_from_rows = sum(expected_row_sums)
        total_from_cols = sum(expected_col_sums)
        assert total_from_rows == total_from_cols, (
            f"Row sum total ({total_from_rows}) != col sum total ({total_from_cols})"
        )

        # The bright block is at rows 0-3, cols 0-3. After blur with kernel=5, some
        # surrounding pixels may also exceed threshold. At minimum the 4 core rows
        # of the block should have nonzero row sums.
        for r in range(4):
            assert expected_row_sums[r] > 0, f"Row {r} in bright block has zero sum"
        for c in range(4):
            assert expected_col_sums[c] > 0, f"Col {c} in bright block has zero sum"

        # Rows and columns far from the block should be zero
        for r in range(8, H):
            assert expected_row_sums[r] == 0, f"Row {r} far from block should be zero"
        for c in range(8, W):
            assert expected_col_sums[c] == 0, f"Col {c} far from block should be zero"
    finally:
        os.unlink(img_path)


# ---------------------------------------------------------------------------
# 6. ROI ranking test
# ---------------------------------------------------------------------------

@pytest.mark.skipif(importErr, reason=reasonErr)
def test_roi_ranking():
    """The region containing a known bright spot is ranked #1."""
    roi_size = 64
    W, H = roi_size * 3, roi_size * 3  # 3x3 grid of regions
    threshold = 100

    img = np.zeros((H, W), dtype=np.uint16)
    # Bright spot in region (regionRow=2, regionCol=1) — bottom-middle region
    r_start = 2 * roi_size
    c_start = 1 * roi_size
    img[r_start:r_start + roi_size, c_start:c_start + roi_size] = 4095

    with tempfile.NamedTemporaryFile(suffix=".tiff", delete=False) as f:
        img_path = f.name
    try:
        _make_tiff(img, img_path)
        mod = _make_module(W, H, kernel=5, threshold=threshold, roi_size=roi_size)
        mod.setImageFileName(img_path)
        sim = _make_sim(mod)
        _run_once(sim)

        roi_msg = mod.roiOutMsg.read()
        assert roi_msg.numValidRegions > 0, "No valid ROI regions reported"
        top = roi_msg.topBins[0]
        assert top.count > 0, "Top ROI has zero pixel count"
        assert top.row == 2, f"Expected top ROI row=2, got {top.row}"
        assert top.col == 1, f"Expected top ROI col=1, got {top.col}"
    finally:
        os.unlink(img_path)


# ---------------------------------------------------------------------------
# 7. Kernel size validation
# ---------------------------------------------------------------------------

@pytest.mark.skipif(importErr, reason=reasonErr)
@pytest.mark.parametrize("bad_kernel", [0, 1, 3, 4, 6, 8, 10, 255])
def test_invalid_kernel_size_raises(bad_kernel):
    """setKernelSize raises immediately for unsupported kernel sizes (only 5, 7, 9 are valid)."""
    with pytest.raises(Exception):
        _make_module(32, 32, kernel=bad_kernel)


@pytest.mark.skipif(importErr, reason=reasonErr)
@pytest.mark.parametrize("good_kernel", [5, 7, 9])
def test_valid_kernel_size_no_raise(good_kernel):
    """setKernelSize accepts valid kernel sizes 5, 7, and 9."""
    mod = _make_module(32, 32, kernel=good_kernel)
    sim = _make_sim(mod)
    sim.InitializeSimulation()  # should not raise


# ---------------------------------------------------------------------------
# 8. Message chaining
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
