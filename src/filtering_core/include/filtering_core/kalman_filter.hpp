// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_KALMAN_FILTER_HPP
#define FILTERING_CORE_KALMAN_FILTER_HPP

#include "measurement_queue.h"

#include <array>
#include <optional>
#include <utility>

namespace filtering {

// A filter is anything that exposes time and measurement updates. We do not
// require a virtual base class; concept-based dispatch lets the queue work
// with any concrete filter type.
//
// `Measurement` may be a single kind or a `std::variant<KindA, KindB, ...>`
// for a multi-sensor filter; in the variant case the filter's
// measurementUpdate std::visit's to a per-kind handler (POD -> model ->
// srukf.update). See filtering_core/_tests/test_heterogeneous_measurements.cpp.
template<class Filter, class Measurement>
concept SequentialFilter = requires(Filter f, Measurement& m, double dt) {
    { f.timeUpdate(dt) };
    { f.measurementUpdate(m) };
};

// Sequential-Kalman-style scheduling: drain `queue` earliest-first; for each
// measurement whose timeTag is in [previousSimSeconds, ∞), advance the filter
// to the measurement's time with `filter.timeUpdate(dt)` and fold it in with
// `filter.measurementUpdate(meas)`; finally a single `filter.timeUpdate` to
// `nextSimSeconds`. The queue is empty on return — each measurement is
// popped as it is consumed, and measurements older than the window start are
// popped and discarded (they can no longer be applied).
template<class Filter, class Measurement, std::size_t Capacity>
    requires SequentialFilter<Filter, Measurement>
void apply_sequential(measurement_queue<Measurement, Capacity>& queue,
                      Filter& filter,
                      double previousSimSeconds,
                      double nextSimSeconds) {
    double currentSimSeconds = previousSimSeconds;

    for (auto entry = queue.popEarliest(); entry.has_value(); entry = queue.popEarliest()) {
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

}  // namespace filtering

#endif
