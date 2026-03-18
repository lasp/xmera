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

    # Minimal — loads image.tiff with default pipeline parameters
    python run_pipeline_reference.py image.tiff

    # Full control
    python run_pipeline_reference.py star_scene.tiff \\
        --width 4096 --height 3000 \\
        --kernel 7 --threshold 200 --roi-size 128

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


def _extract_roi(mod, log_roi_out_msg):
    numRegions = int(log_roi_out_msg.numValidRegions)
    regions = []
    for rank in range(numRegions):
        entry = log_roi_out_msg.topRegions[rank]
        regions.append({
            "rank": rank,
            "row": int(entry.row),
            "col": int(entry.col),
            "count": int(entry.count),
        })
    return int(log_roi_out_msg.regionSize), regions


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
    parser.add_argument("--threshold", type=int,   default=21000,    help="Threshold value (0–65535)")
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
    region_size, roi_regions = _extract_roi(mod, log_roi_out_msg)

    # --- Save ---
    _save_tiff(raw,    os.path.join(_DATA_DIR, "raw_calibrated.tiff"))
    _save_tiff(blur,   os.path.join(_DATA_DIR, "blurred.tiff"))
    _save_png(thresh,  os.path.join(_DATA_DIR, "threshold.png"))
    _save_csv_vector(row_sums, "row",    os.path.join(_DATA_DIR, "row_sums.csv"))
    _save_csv_vector(col_sums, "col",    os.path.join(_DATA_DIR, "col_sums.csv"))
    _save_roi_csv(roi_regions, region_size, os.path.join(_DATA_DIR, "roi.csv"))

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


if __name__ == "__main__":
    main()
