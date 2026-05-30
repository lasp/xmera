#include "regionsOfInterestPruneAlgorithm.h"

#include <algorithm>
#include <functional>
#include <numeric>

/*! Update method for the regions-of-interest pruning algorithm.  Orchestrates
 *  the four pipeline steps and returns a fully populated RoiCandidates.
 @return RoiCandidates  Candidates sorted by estimated pixel count (descending).
 @param rowSums   Pointer to the per-row above-threshold pixel sums.
 @param numRows   Number of rows in the sum array.
 @param colSums   Pointer to the per-column above-threshold pixel sums.
 @param numCols   Number of columns in the sum array.
*/
RoiCandidates RegionsOfInterestPruneAlgorithm::update(const uint16_t* rowSums,
                                                      uint32_t numRows,
                                                      const uint16_t* colSums,
                                                      uint32_t numCols) const {
    // Step 1: locate contiguous non-zero spans and accumulate per-span pixel sums.
    const auto [rowSpans, rowAccum] = findSpans(rowSums, numRows);
    const auto [colSpans, colAccum] = findSpans(colSums, numCols);

    // Step 2: keep only the highest-sum spans to bound the cross-product size.
    const auto topRows = topIndices(rowAccum, this->maxRowSpans);
    const auto topCols = topIndices(colAccum, this->maxColSpans);

    // Steps 3–4: form bounding boxes, sort by estimated pixel count, truncate.
    return packOutput(buildCandidates(rowSpans, rowAccum, topRows, colSpans, colAccum, topCols));
}

/*! Scans a 1-D sum array and returns all contiguous non-zero spans together
 *  with the accumulated sum for each span.  A single forward pass produces both.
 @return Pair of (spans, accum) where spans[i] = (start, length) and accum[i] = sum over that span.
 @param s  Pointer to the 1-D sum array.
 @param n  Length of the array.
*/
std::pair<RegionsOfInterestPruneAlgorithm::SpanVec, RegionsOfInterestPruneAlgorithm::AccumVec>
RegionsOfInterestPruneAlgorithm::findSpans(const uint16_t* s, uint32_t n) {
    SpanVec spans;
    AccumVec accum;
    for (uint32_t i = 0; i < n;) {
        if (s[i]) {
            uint32_t j = i;
            uint32_t sum = 0;
            while (j < n && s[j]) sum += s[j++];
            spans.emplace_back(i, j - i);
            accum.push_back(sum);
            i = j;
        } else
            ++i;
    }
    return {spans, accum};
}

/*! Returns the indices of the top-keep entries of vals ordered by descending value.
 *  Uses std::partial_sort so only the retained portion is fully sorted (O(N log keep)).
 @return Vector of at most keep indices into vals, sorted by vals[i] descending.
 @param vals  Accumulator values to rank.
 @param keep  Maximum number of top entries to retain.
*/
RegionsOfInterestPruneAlgorithm::AccumVec RegionsOfInterestPruneAlgorithm::topIndices(const AccumVec& vals,
                                                                                      uint32_t keep) {
    AccumVec idx(vals.size());
    std::iota(idx.begin(), idx.end(), 0);
    keep = std::min(keep, static_cast<uint32_t>(vals.size()));
    std::ranges::partial_sort(idx, idx.begin() + keep, std::greater{}, [&vals](uint32_t i) { return vals[i]; });
    idx.resize(keep);
    return idx;
}

/*! Forms bounding-box candidates from the cross-product of the filtered row and col spans.
 *  The estimated pixel count for each box is min(R[k], C[l]) — the tightest upper bound
 *  obtainable from 1-D projections alone.
 @return Vector of RoiCandidateEntry, one per (rowIdx × colIdx) pair.
 @param rowSpans  All detected row spans.
 @param R         Per-row-span accumulator sums.
 @param rowIdx    Indices into rowSpans / R selected by the pre-filter (Step 2).
 @param colSpans  All detected col spans.
 @param C         Per-col-span accumulator sums.
 @param colIdx    Indices into colSpans / C selected by the pre-filter (Step 2).
*/
std::vector<RoiCandidateEntry> RegionsOfInterestPruneAlgorithm::buildCandidates(const SpanVec& rowSpans,
                                                                                const AccumVec& R,
                                                                                const AccumVec& rowIdx,
                                                                                const SpanVec& colSpans,
                                                                                const AccumVec& C,
                                                                                const AccumVec& colIdx) {
    std::vector<RoiCandidateEntry> candidates;
    for (uint32_t ki : rowIdx)
        for (uint32_t li : colIdx) {
            const auto [r, h] = rowSpans[ki];
            const auto [c, w] = colSpans[li];
            candidates.push_back({r, c, h, w, std::min(R[ki], C[li])});
        }
    return candidates;
}

/*! Sorts candidates by estimated pixel count (descending), use window area for tie break, truncates to
 ROI_CANDIDATES_MAX,
 *  and packs the result into a RoiCandidates ready for publication.
 @return RoiCandidates with numCandidates set and candidates[0] = rank-1.
 @param candidates  Unsorted candidate list (taken by value; sorted in-place).
*/
RoiCandidates RegionsOfInterestPruneAlgorithm::packOutput(std::vector<RoiCandidateEntry> candidates) {
    std::ranges::sort(candidates, [](const RoiCandidateEntry& a, const RoiCandidateEntry& b) {
        if (a.count != b.count) return a.count > b.count;
        return a.height * a.width < b.height * b.width;
    });
    if (candidates.size() > ROI_CANDIDATES_MAX) candidates.resize(ROI_CANDIDATES_MAX);

    RoiCandidates outRoi{};
    outRoi.numCandidates = static_cast<uint32_t>(candidates.size());
    for (uint32_t i = 0; i < outRoi.numCandidates; ++i) outRoi.candidates[i] = candidates[i];
    return outRoi;
}
