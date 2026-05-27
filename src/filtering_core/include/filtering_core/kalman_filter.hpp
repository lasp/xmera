// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_KALMAN_FILTER_HPP
#define FILTERING_CORE_KALMAN_FILTER_HPP

#include <array>
#include <cstddef>
#include <optional>
#include <utility>

namespace filtering {

// A filter is anything that exposes time and measurement updates. We do not
// require a virtual base class; concept-based dispatch lets the queue work
// with any concrete filter type.
template<class Filter, class Measurement>
concept Updateable = requires(Filter f, Measurement& m, double dt) {
    { f.timeUpdate(dt) };
    { f.measurementUpdate(m) };
};

// Bounded-capacity, time-ordered queue of measurements, applied to a filter
// in chronological order during a single update window.
//
// `applyToFilter(filter, t0, t1)` calls:
//   - filter.timeUpdate(meas.timeTag - currentTime)
//   - filter.measurementUpdate(meas)
//   ... once per queued measurement whose timeTag is in [t0, t1], then a
// final filter.timeUpdate to bring the filter to t1.
//
// Measurements are popped as they're applied (the queue empties in the
// course of `applyToFilter`).
template<typename Measurement, std::size_t CAPACITY>
class measurement_queue final {
public:
    bool isEmpty() const { return this->size == 0; }
    bool isFull()  const { return this->size >= CAPACITY; }

    bool enqueue(double timeTag, Measurement&& measurement) {
        if (this->isFull()) return false;

        std::size_t insertionIndex = this->size;
        while (insertionIndex > 0) {
            if (timeTag <= this->measurements[insertionIndex - 1].value().first) break;

            this->measurements[insertionIndex] =
                std::move(this->measurements[insertionIndex - 1]);

            insertionIndex -= 1;
        }

        this->measurements[insertionIndex] = {timeTag, std::move(measurement)};
        this->size += 1;
        return true;
    }

    void clear() {
        while (this->size > 0) {
            this->size -= 1;
            this->measurements[this->size] = std::nullopt;
        }
    }

    std::optional<std::pair<double, Measurement>> popEarliest() {
        if (this->isEmpty()) return std::nullopt;

        this->size -= 1;
        return std::exchange(this->measurements[this->size], std::nullopt);
    }

    template<class Filter>
        requires Updateable<Filter, Measurement>
    void applyToFilter(Filter& filter, double previousSimSeconds, double nextSimSeconds) {
        double currentSimSeconds = previousSimSeconds;

        // Drain the queue earliest-first (popEarliest yields ascending time).
        // Each measurement is removed as it is consumed, so the queue is empty
        // when applyToFilter returns. Measurements older than the window start
        // are popped and discarded (they can no longer be applied).
        for (auto entry = this->popEarliest(); entry.has_value(); entry = this->popEarliest()) {
            auto& [timeTag, measurement] = entry.value();
            if (timeTag < currentSimSeconds) continue;

            filter.timeUpdate(timeTag - currentSimSeconds);
            filter.measurementUpdate(measurement);

            currentSimSeconds = timeTag;
        }

        if (currentSimSeconds < nextSimSeconds) {
            filter.timeUpdate(nextSimSeconds - currentSimSeconds);
        }
    }

private:
    // INVARIANT: `measurements[i].has_value() == (i < size)`
    //   That is, all initialized values appear before all uninitialized values.
    // INVARIANT: `measurements[i + 1].value().first <= measurements[i].value().first`
    //   That is, larger time tags come earlier in the storage array.
    std::size_t size = 0;
    std::array<std::optional<std::pair<double, Measurement>>, CAPACITY> measurements = {};
};

}  // namespace filtering

#endif
