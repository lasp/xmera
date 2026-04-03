"""run_pipeline_reference.py — FPGA image pipeline reference runner.

Loads a raw 16-bit grayscale TIFF, runs it through the FpgaImagePipeline
simulation module, and writes every pipeline product to a ``data/``
directory beside this script.

Outputs written to ``data/``:
    raw_calibrated.tiff     — calibration-preprocessed image (uint16)
    blurred.tiff            — box-blurred image (uint16)
    threshold.png           — binary threshold image (uint8, 0 or 255)
    row_sums.csv            — above-threshold pixel count per row
    col_sums.csv            — above-threshold pixel count per column
    roi.csv                 — top ROI regions (rank, row, col, count)

Usage examples::

    # Full control
    python run_pipeline_reference.py test_cells.tif --width 4096 --height 3000 --kernel 7 --roi-size 128
    python run_pipeline_reference.py pia_958_830.tiff --width 958 --height 830 --kernel 5 --roi-size 128
    python run_pipeline_reference.py pia_616_592.tif --width 616 --height 592 --kernel 5 --roi-size 128
    python run_pipeline_reference.py pia_1920_1080.tif --width 1920 --height 1080 --kernel 9 --roi-size 128

    # Enable built-in module disk saves as well
    python run_pipeline_reference.py image.tiff --module-save
"""

import argparse
import csv
import inspect
import os
import sys

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Path setup — allow running from any working directory
# ---------------------------------------------------------------------------

_HERE = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
_DATA_DIR = os.path.join(_HERE, "data")

# ---------------------------------------------------------------------------
# Imports (give a helpful message if the module is not built)
# ---------------------------------------------------------------------------

try:
    from xmera.fswAlgorithms import fpgaImagePipeline
except ImportError as e:
    sys.exit(
        f"ERROR: could not import fpgaImagePipeline — is the module built?\n"
        f"  {e}"
    )

from xmera.utilities import SimulationBaseClass, macros
from xmera.architecture import messaging

# ---------------------------------------------------------------------------
# Simulation helpers
# ---------------------------------------------------------------------------

def _build_module(args):
    """Construct and configure an FpgaImagePipeline module from parsed args."""
    mod = fpgaImagePipeline.FpgaImagePipeline()
    mod.ModelTag = "pipelineRef"
    mod.setImageWidth(args.width)
    mod.setImageHeight(args.height)
    mod.setKernelSize(args.kernel)
    mod.setThreshold(args.threshold)
    mod.setRoiRegionSize(args.roi_size)
    mod.setImageFileName(args.image)

    if args.calib_image:
        mod.setCalibEnabled(True)
        mod.setCalibImageFile(args.calib_image)
        mod.setCalibRegA(args.calib_a)
        mod.setCalibRegB(args.calib_b)
        mod.setCalibRegC(args.calib_c)
        mod.setCalibRegD(args.calib_d)

    if args.module_save:
        mod.setSaveImages(True)
        mod.setSaveDir(_DATA_DIR)

    return mod


# ---------------------------------------------------------------------------
# Product extraction helpers
# ---------------------------------------------------------------------------

def _extract_raw(mod, W, H):
    """Read calibrated raw pixel buffer into a (H, W) uint16 array."""
    arr = np.empty(H * W, dtype=np.uint16)
    for i in range(H * W):
        arr[i] = mod.getRawPixel(i)
    return arr.reshape(H, W)


def _extract_blur(mod, W, H):
    """Read blurred pixel buffer into a (H, W) uint16 array."""
    arr = np.empty(H * W, dtype=np.uint16)
    for i in range(H * W):
        arr[i] = mod.getBlurPixel(i)
    return arr.reshape(H, W)


def _extract_thresh(mod, W, H):
    """Reconstruct threshold image as a (H, W) uint8 array (0 or 255)."""
    arr = np.zeros(H * W, dtype=np.uint8)
    for i in range(H * W):
        if mod.getThreshBit(i):
            arr[i] = 255
    return arr.reshape(H, W)


def _extract_row_sums(mod, H):
    return [mod.getRowSum(r) for r in range(H)]


def _extract_col_sums(mod, W):
    return [mod.getColSum(c) for c in range(W)]


def _extract_roi(log_roi_out_msg):
    numRegions = log_roi_out_msg.numValidRegions[0]
    topBins_data = log_roi_out_msg.topBins[0]  # keys: "topBins[k].row" etc.
    regions = []
    for rank in range(numRegions):
        regions.append({
            "rank": rank,
            "row": int(topBins_data[f"topBins[{rank}].row"]),
            "col": int(topBins_data[f"topBins[{rank}].col"]),
            "count": int(topBins_data[f"topBins[{rank}].count"]),
        })
    return log_roi_out_msg.regionSize[0], regions


# ---------------------------------------------------------------------------
# Save helpers
# ---------------------------------------------------------------------------

def _save_tiff(arr, path):
    cv2.imwrite(path, arr)


def _save_png(arr, path):
    cv2.imwrite(path, arr)


def _save_csv_vector(values, header, path):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([header, "count"])
        for i, v in enumerate(values):
            writer.writerow([i, v])


def _save_roi_csv(regions, region_size, path):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["rank", "region_row", "region_col",
                          "pixel_row", "pixel_col", "above_threshold_count"])
        for r in regions:
            writer.writerow([
                r["rank"],
                r["row"],
                r["col"],
                r["row"] * region_size,
                r["col"] * region_size,
                r["count"],
            ])


def _save_roi_overlay_tiff(blur, roi_regions, region_size, path):
    """Save a colour-annotated TIFF with the top ROI regions drawn on the blur image.

    Each of the up-to-8 valid regions is drawn as a coloured rectangle whose
    top-left corner is at (pixel_col, pixel_row) = (region.col * region_size,
    region.row * region_size).  The rank number (0 = best) is printed inside
    the rectangle.  Rank 0 is red; subsequent ranks cycle through a fixed palette.

    The blur image is normalised to 8-bit [0, 255] so that the annotation is
    visible regardless of the blur output scale.
    """
    # Colour palette (BGR): rank 0 → red, 1 → orange, 2 → yellow, 3 → green,
    #                        4 → cyan, 5 → blue, 6 → magenta, 7 → white
    rank_colors = [
        (0,   0,   255),
        (0,   128, 255),
        (0,   255, 255),
        (0,   255,   0),
        (255, 255,   0),
        (255,   0,   0),
        (255,   0, 255),
        (255, 255, 255),
    ]

    # Normalise blur to 8-bit and convert to BGR for colour drawing
    blur_8bit = cv2.normalize(blur, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    vis = cv2.cvtColor(blur_8bit, cv2.COLOR_GRAY2BGR)

    for r in roi_regions:
        rank   = r["rank"]
        px_row = r["row"] * region_size
        px_col = r["col"] * region_size
        color  = rank_colors[rank % len(rank_colors)]
        pt1 = (px_col, px_row)
        pt2 = (px_col + region_size - 1, px_row + region_size - 1)
        cv2.rectangle(vis, pt1, pt2, color, 2)
        cv2.putText(vis, str(rank), (px_col + 4, px_row + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

    cv2.imwrite(path, vis)


# ---------------------------------------------------------------------------
# Bit-depth detection and auto-threshold
# ---------------------------------------------------------------------------

def _detect_image_info(image_path, kernel):
    """Read the image header to detect bit depth and suggest a pipeline threshold.

    Bit-depth detection
    -------------------
    The module internally converts 8-bit images by scaling pixel values ×16
    (bringing [0, 255] into the 12-bit equivalent range [0, 4080]).  This
    function replicates that scaling so the suggested threshold is always
    expressed in blur-output units, regardless of the source depth.

    Threshold suggestion
    --------------------
    The pipeline threshold is compared against *blur* output values, not raw
    pixel values.  For a uniform bright patch of intensity V with kernel k:

        blur = (V * k * k) >> blurShift(k)    (blurShift: 5→1, 7→2, 9→3)

    The suggested threshold is set to 70% of the blur value at the 99th
    percentile raw pixel intensity.  This ensures that genuinely bright regions
    (top 1% of pixels) will be above threshold after blurring, while typical
    background pixels are not.

    Returns
    -------
    (is_8bit : bool, width : int, height : int, suggested_threshold : int)
    """
    img = cv2.imread(image_path, cv2.IMREAD_ANYDEPTH | cv2.IMREAD_GRAYSCALE)
    if img is None:
        return False, None, None, 21000

    is_8bit = (img.dtype == np.uint8)
    img16 = img.astype(np.uint16) * 16 if is_8bit else img.astype(np.uint16)

    h, w = img16.shape
    shift = {5: 1, 7: 2, 9: 3}.get(int(kernel), 1)
    p99 = int(np.percentile(img16.ravel(), 99))
    blur_p99 = (p99 * int(kernel) * int(kernel)) >> shift
    suggested = max(1, int(blur_p99 * 0.7))

    return is_8bit, w, h, suggested


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Run the FPGA image pipeline on a reference image and save all products.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("image", help="Path to 16-bit grayscale input TIFF")
    parser.add_argument("--width",     type=int,   default=4096, help="Image width in pixels")
    parser.add_argument("--height",    type=int,   default=3000, help="Image height in pixels")
    parser.add_argument("--kernel",    type=int,   default=5,    help="Blur kernel size (5, 7, or 9)")
    parser.add_argument("--threshold", type=int,   default=None,
                        help="Threshold value (0–65535). Omit to auto-detect from image.")
    parser.add_argument("--roi-size",  type=int,   default=64,   help="ROI region size (64, 128, or 256)")

    calib = parser.add_argument_group("calibration (optional)")
    calib.add_argument("--calib-image", default=None, help="Path to 16-bit calibration TIFF")
    calib.add_argument("--calib-a", type=int, default=0, help="Calibration register A")
    calib.add_argument("--calib-b", type=int, default=0, help="Calibration register B")
    calib.add_argument("--calib-c", type=int, default=0, help="Calibration register C")
    calib.add_argument("--calib-d", type=int, default=0, help="Calibration register D")

    parser.add_argument(
        "--module-save", action="store_true",
        help="Also invoke the module's built-in save (setSaveImages/setSaveDir)"
    )

    args = parser.parse_args()

    if not os.path.isfile(args.image):
        sys.exit(f"ERROR: input image not found: {args.image}")

    os.makedirs(_DATA_DIR, exist_ok=True)

    # --- Bit-depth detection and auto-threshold ---
    is_8bit, detected_w, detected_h, auto_threshold = _detect_image_info(args.image, args.kernel)
    depth_str = "8-bit  (module will scale ×16 → 12-bit equivalent)" if is_8bit else "16-bit"
    print(f"Image depth       : {depth_str}")
    if detected_w is not None:
        print(f"Detected size     : {detected_w}×{detected_h}  (configured: {args.width}×{args.height})")
    if args.threshold is None:
        args.threshold = auto_threshold
        print(f"Auto-threshold    : {args.threshold}  (70% of blur at 99th-percentile pixel)")
    else:
        print(f"Threshold         : {args.threshold}  (user-specified)")

    W, H = args.width, args.height

    # --- Build and run ---
    print(f"Input  : {args.image}  ({W}x{H})")
    print(f"Kernel : {args.kernel}   Threshold: {args.threshold}   ROI size: {args.roi_size}")
    print(f"Output : {_DATA_DIR}")

    mod = _build_module(args)
    log_roi_out_msg = mod.roiOutMsg.recorder()
    sim = SimulationBaseClass.SimBaseClass()
    testProc = sim.CreateNewProcess("refProcess")
    testProc.addTask(sim.CreateNewTask("refTask", macros.sec2nano(1.0)))
    sim.AddModelToTask("refTask", mod)
    sim.AddModelToTask("refTask", log_roi_out_msg)
    sim.InitializeSimulation()
    sim.TotalSim.singleStepProcesses()

    # --- Extract ---
    raw    = _extract_raw(mod, W, H)
    blur   = _extract_blur(mod, W, H)
    thresh = _extract_thresh(mod, W, H)
    row_sums = _extract_row_sums(mod, H)
    col_sums = _extract_col_sums(mod, W)
    region_size, roi_regions = _extract_roi(log_roi_out_msg)

    # --- Save ---
    _save_tiff(raw,    os.path.join(_DATA_DIR, "raw_calibrated.tiff"))
    _save_tiff(blur,   os.path.join(_DATA_DIR, "blurred.tiff"))
    _save_png(thresh,  os.path.join(_DATA_DIR, "threshold.png"))
    _save_csv_vector(row_sums, "row",    os.path.join(_DATA_DIR, "row_sums.csv"))
    _save_csv_vector(col_sums, "col",    os.path.join(_DATA_DIR, "col_sums.csv"))
    _save_roi_csv(roi_regions, region_size, os.path.join(_DATA_DIR, "roi.csv"))
    _save_roi_overlay_tiff(blur, roi_regions, region_size, os.path.join(_DATA_DIR, "roi_overlay.tiff"))

    # --- Summary ---
    above = int(thresh.astype(bool).sum())
    print(f"\nPipeline complete:")
    print(f"  Above-threshold pixels : {above}")
    print(f"  Valid ROI regions      : {len(roi_regions)}")
    if roi_regions:
        top = roi_regions[0]
        print(f"  Top ROI (rank 0)       : region ({top['row']}, {top['col']})"
              f"  pixel ({top['row'] * region_size}, {top['col'] * region_size})"
              f"  count={top['count']}")
    print(f"\nProducts saved to {_DATA_DIR}/")
    for name in ("raw_calibrated.tiff", "blurred.tiff", "threshold.png",
                 "row_sums.csv", "col_sums.csv", "roi.csv"):
        print(f"  {name}")
    print(f"  roi_overlay.tiff")


if __name__ == "__main__":
    main()
