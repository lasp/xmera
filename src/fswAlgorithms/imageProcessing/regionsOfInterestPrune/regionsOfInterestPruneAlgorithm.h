#ifndef REGIONS_OF_INTEREST_PRUNE_ALGORITHM_H
#define REGIONS_OF_INTEREST_PRUNE_ALGORITHM_H

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

/*! @brief Internal bounding-box candidate (corner coordinates + pixel count). */
static constexpr uint32_t ROI_CANDIDATES_MAX = 16;

struct RoiCandidateEntry {
    uint32_t row{};     //!< [px] Top-left row of the bounding box
    uint32_t col{};     //!< [px] Top-left column of the bounding box
    uint32_t height{};  //!< [px] Height of the bounding box
    uint32_t width{};   //!< [px] Width of the bounding box
    uint32_t count{};   //!< [-] Estimated above-threshold pixel count
};

struct RoiCandidates {
    uint32_t numCandidates{};                                      //!< [-] Number of valid entries
    std::array<RoiCandidateEntry, ROI_CANDIDATES_MAX> candidates;  //!< [-] Sorted by count descending
};

/*! @brief algorithm for the regions-of-interest pruning stage.
 *
 *  Accepts raw row/col sum arrays and returns a
 *  RoiCandidates.  Pipeline stages:
 *    1. Find contiguous non-zero spans in rowSums / colSums; accumulate per-span sums.
 *    2. Pre-filter to top maxRowSpans / maxColSpans spans by accumulator value.
 *    3. Cross-product of filtered spans → bounding boxes with count = min(R[k], C[l]).
 *    4. Sort by count descending, return top ROI_CANDIDATES_MAX entries.
 */
class RegionsOfInterestPruneAlgorithm {
   public:
    RoiCandidates update(const uint16_t* rowSums, uint32_t numRows, const uint16_t* colSums, uint32_t numCols) const;

    void setMaxRowSpans(uint32_t n) { this->maxRowSpans = n; }
    uint32_t getMaxRowSpans() const { return this->maxRowSpans; }
    void setMaxColSpans(uint32_t n) { this->maxColSpans = n; }
    uint32_t getMaxColSpans() const { return this->maxColSpans; }

   private:
    uint32_t maxRowSpans{3};  //!< Pre-filter: keep this many top row spans before cross-product
    uint32_t maxColSpans{3};  //!< Pre-filter: keep this many top col spans before cross-product

    using Span = std::pair<uint32_t, uint32_t>;
    using SpanVec = std::vector<Span>;
    using AccumVec = std::vector<uint32_t>;

    // Step 1: find contiguous non-zero spans and accumulate per-span sums.
    static std::pair<SpanVec, AccumVec> findSpans(const uint16_t* s, uint32_t n);

    // Step 2: return indices of the top-keep entries in vals (by descending value).
    static AccumVec topIndices(const AccumVec& vals, uint32_t keep);

    // Step 3: cross-product of filtered row/col spans → bounding-box candidates.
    static std::vector<RoiCandidateEntry> buildCandidates(const SpanVec& rowSpans,
                                                          const AccumVec& R,
                                                          const AccumVec& rowIdx,
                                                          const SpanVec& colSpans,
                                                          const AccumVec& C,
                                                          const AccumVec& colIdx);

    // Step 4: sort candidates by count descending, truncate, and pack into a message.
    static RoiCandidates packOutput(std::vector<RoiCandidateEntry> candidates);
};

#endif
