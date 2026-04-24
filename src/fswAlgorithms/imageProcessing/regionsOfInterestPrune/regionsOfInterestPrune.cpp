#include "regionsOfInterestPrune.h"

#include <string>

// ---------------------------------------------------------------------------
// Free helper — converts a single internal candidate (corner-based) to the
// center-coordinate form expected by RegionsIdentifiedMsgPayload.
// ---------------------------------------------------------------------------
static RegionOfInterestMsgPayload toRegionOfInterest(const RoiCandidateEntry& cand, double timeTagSec) {
    RegionOfInterestMsgPayload reg{};
    reg.timeTag = timeTagSec;
    reg.centerX = static_cast<int>(cand.col + cand.width / 2);
    reg.centerY = static_cast<int>(cand.row + cand.height / 2);
    reg.width = static_cast<int>(cand.width);
    reg.height = static_cast<int>(cand.height);
    reg.numberOfPixels = static_cast<int>(cand.count);
    reg.centerOfBrightnessX = reg.centerX;  // best estimate without full pixel data
    reg.centerOfBrightnessY = reg.centerY;
    return reg;
}

// ---------------------------------------------------------------------------
// BSK adapter interface
// ---------------------------------------------------------------------------

void RegionsOfInterestPrune::reset(uint64_t /*callTime*/) {
    if (!rowColSumInMsg.isLinked())
        bskLogger.bskLog(BSK_WARNING, "RegionsOfInterestPrune: rowColSumInMsg is not linked");
    numPublished = 0;
    lastRegionsOutput = {};
}

void RegionsOfInterestPrune::updateState(uint64_t callTime) {
    if (!rowColSumInMsg.isLinked()) return;

    const FpgaRowColSumMsgPayload rcMsg = rowColSumInMsg();
    const auto* rowSums = reinterpret_cast<const uint16_t*>(rcMsg.rowSumPointer);
    const auto* colSums = reinterpret_cast<const uint16_t*>(rcMsg.colSumPointer);

    const RoiCandidates candidates = algorithm.update(rowSums, rcMsg.numRows, colSums, rcMsg.numCols);

    lastRegionsOutput = {};
    numPublished = std::min(candidates.numCandidates, static_cast<uint32_t>(MAX_NUMBER_REGIONS));
    const double timeTagSec = static_cast<double>(callTime) * NANO2SEC;
    for (uint32_t k = 0; k < numPublished; ++k)
        lastRegionsOutput.regions[k] = toRegionOfInterest(candidates.candidates[k], timeTagSec);
    regionsIdentifiedOutMsg.write(&lastRegionsOutput, moduleID, callTime);

    if (saveImages && !saveDir.empty()) saveVisualization(rcMsg);
}

// ---------------------------------------------------------------------------
// Visualization helpers
// ---------------------------------------------------------------------------

/*! Builds a BGR background image for the current frame.
 *  Prefers the packed 1-bit threshold image; falls back to a synthetic image
 *  whose brightness at (r,c) is min(rowSum[r], colSum[c]).
 @return BGR cv::Mat of size H×W, or an empty Mat on failure.
 @param rcMsg  Row/col sum message (carries dimensions and fallback data).
*/
cv::Mat RegionsOfInterestPrune::buildBackground(const FpgaRowColSumMsgPayload& rcMsg) {
    const auto H = static_cast<int>(rcMsg.numRows);
    const auto W = static_cast<int>(rcMsg.numCols);

    if (threshImageInMsg.isLinked()) {
        const FpgaThreshImageMsgPayload thMsg = threshImageInMsg();
        const auto* bits = reinterpret_cast<const uint8_t*>(thMsg.imagePointer);
        if (bits && thMsg.width == static_cast<uint32_t>(W) && thMsg.height == static_cast<uint32_t>(H)) {
            // Unpack 1-bit-per-pixel (MSB-first) → 8-bit grayscale → BGR.
            cv::Mat gray(H, W, CV_8UC1);
            const int nPix = H * W;
            for (int i = 0; i < nPix; ++i)
                gray.at<uint8_t>(i / W, i % W) = static_cast<uint8_t>(((bits[i / 8] >> (7 - i % 8)) & 1) * 255);
            cv::Mat bgr;
            cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
            return bgr;
        }
    }

    // Fallback: synthesize from 1-D sums.
    const auto* rowSums = reinterpret_cast<const uint16_t*>(rcMsg.rowSumPointer);
    const auto* colSums = reinterpret_cast<const uint16_t*>(rcMsg.colSumPointer);
    if (!rowSums || !colSums) return {};

    uint32_t maxVal = 0;
    for (int r = 0; r < H; ++r) maxVal = std::max(maxVal, static_cast<uint32_t>(rowSums[r]));
    for (int c = 0; c < W; ++c) maxVal = std::max(maxVal, static_cast<uint32_t>(colSums[c]));

    cv::Mat gray(H, W, CV_8UC1, cv::Scalar(0));
    if (maxVal > 0) {
        for (int r = 0; r < H; ++r)
            for (int c = 0; c < W; ++c)
                gray.at<uint8_t>(r, c) =
                    static_cast<uint8_t>(std::min<uint32_t>(rowSums[r], colSums[c]) * 255u / maxVal);
    }
    cv::Mat bgr;
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
}

/*! Annotates one ranked region on vis: thick bounding box + filled center dot + label.
 @param vis        BGR image to annotate in-place.
 @param reg        Region in center-coordinate form.
 @param color      BGR annotation color.
 @param thickness  Bounding-box line thickness.
 @param label      Text label drawn above the box.
*/
void RegionsOfInterestPrune::drawRegion(cv::Mat& vis,
                                        const RegionOfInterestMsgPayload& reg,
                                        const cv::Scalar& color,
                                        int thickness,
                                        const std::string& label) {
    const int x = reg.centerX - reg.width / 2;
    const int y = reg.centerY - reg.height / 2;
    cv::rectangle(vis, cv::Point(x, y), cv::Point(x + reg.width, y + reg.height), color, thickness);
    cv::circle(vis, cv::Point(reg.centerX, reg.centerY), 3, color, -1);
    cv::putText(vis, label, cv::Point(x, std::max(y - 6, 12)), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
}

void RegionsOfInterestPrune::saveVisualization(const FpgaRowColSumMsgPayload& rcMsg) {
    if (rcMsg.numRows == 0 || rcMsg.numCols == 0) return;

    cv::Mat vis = buildBackground(rcMsg);
    if (vis.empty()) return;

    const uint32_t nDraw = numPublished;

    // All published regions: thin cyan outline for context.
    for (uint32_t k = 0; k < nDraw; ++k) {
        const auto& reg = lastRegionsOutput.regions[k];
        const int x = reg.centerX - reg.width / 2;
        const int y = reg.centerY - reg.height / 2;
        cv::rectangle(vis, cv::Point(x, y), cv::Point(x + reg.width, y + reg.height), cv::Scalar(0, 255, 255), 1);
    }
    // Rank-1 (red) and rank-2 (blue): thicker box + center dot + pixel-count label.
    static const cv::Scalar kRed(0, 0, 255);
    static const cv::Scalar kBlue(255, 0, 0);
    if (nDraw >= 1) {
        const auto& r1 = lastRegionsOutput.regions[0];
        drawRegion(vis, r1, kRed, 2, "R1 (" + std::to_string(r1.numberOfPixels) + ")");
    }
    if (nDraw >= 2) {
        const auto& r2 = lastRegionsOutput.regions[1];
        drawRegion(vis, r2, kBlue, 2, "R2 (" + std::to_string(r2.numberOfPixels) + ")");
    }

    cv::imwrite(saveDir + "/" + std::to_string(rcMsg.timeTag) + "_pruning_output.png", vis);
}
