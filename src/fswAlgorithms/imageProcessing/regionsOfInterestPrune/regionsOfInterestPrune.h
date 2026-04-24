#ifndef REGIONS_OF_INTEREST_PRUNE_H
#define REGIONS_OF_INTEREST_PRUNE_H

#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/FpgaRowColSumMsgPayload.h>
#include <architecture/msgPayloadDef/FpgaThreshImageMsgPayload.h>
#include <architecture/msgPayloadDef/RegionsIdentifiedMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/macroDefinitions.h>

#include "regionsOfInterestPruneAlgorithm.h"

/*! @brief Basilisk adapter for the regions-of-interest pruning module.
 *
 *  Reads FpgaRowColSumMsgPayload, delegates computation to RegionsOfInterestPruneAlgorithm,
 *  and publishes up to MAX_NUMBER_REGIONS candidates sorted by estimated above-threshold
 *  pixel count to regionsIdentifiedOutMsg (RegionsIdentifiedMsgPayload).
 */
class RegionsOfInterestPrune : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    // --- Pre-filter configuration (forwarded to algorithm) ---
    void setMaxRowSpans(uint32_t n) { algorithm.setMaxRowSpans(n); }
    uint32_t getMaxRowSpans() const { return algorithm.getMaxRowSpans(); }
    void setMaxColSpans(uint32_t n) { algorithm.setMaxColSpans(n); }
    uint32_t getMaxColSpans() const { return algorithm.getMaxColSpans(); }

    // --- Save configuration ---
    void setSaveImages(bool save) { saveImages = save; }
    bool getSaveImages() const { return saveImages; }
    void setSaveDir(const std::string& dir) { saveDir = dir; }
    std::string getSaveDir() const { return saveDir; }

    // --- Message interfaces ---
    ReadFunctor<FpgaRowColSumMsgPayload> rowColSumInMsg;           //!< Row/col accumulators from fpgaImagePipeline
    ReadFunctor<FpgaThreshImageMsgPayload> threshImageInMsg;       //!< Optional: threshold image for visualization
    Message<RegionsIdentifiedMsgPayload> regionsIdentifiedOutMsg;  //!< Pruned candidates as RegionsIdentifiedMsg

    BSKLogger bskLogger = {};

   private:
    // Build the BGR background image from the threshold msg (preferred) or the 1-D sums (fallback).
    cv::Mat buildBackground(const FpgaRowColSumMsgPayload& rcMsg);
    // Draw a thick bounding-box + filled center dot + label for one ranked region.
    static void drawRegion(cv::Mat& vis,
                           const RegionOfInterestMsgPayload& reg,
                           const cv::Scalar& color,
                           int thickness,
                           const std::string& label);
    void saveVisualization(const FpgaRowColSumMsgPayload& rcMsg);

    RegionsOfInterestPruneAlgorithm algorithm{};
    uint32_t numPublished{};                          //!< Number of valid entries in lastRegionsOutput
    RegionsIdentifiedMsgPayload lastRegionsOutput{};  //!< Published center-coordinate form
    bool saveImages{false};
    std::string saveDir{};
};

#endif
