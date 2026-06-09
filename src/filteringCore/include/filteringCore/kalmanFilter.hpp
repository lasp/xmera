// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_KALMAN_FILTER_HPP
#define FILTERING_CORE_KALMAN_FILTER_HPP

#include "measurementQueue.h"

namespace filtering {

/*! A sequential filter exposes the time-update and measurement-update functions. */
template<class Filter, class Measurement>
concept SequentialFilter = requires(Filter f, Measurement& m, double dt) {
    { f.timeUpdate(dt) };
    { f.measurementUpdate(m) };
};

/*! Sequential filter scheduling: empty the measurement queue in chronological order, calling
 *  timeUpdate from the last measurement-updated state to the measurement timeTag then measurementUpdate
 *  for each measurement; Call timeUpdate to `callTime` from there so the filter's `state`
 *  reflects the call time on return.
 *  @param queue    [-] measurement queue (drained on return)
 *  @param filter   [-] filter satisfying SequentialFilter
 *  @param callTime [s] sim time the filter is advancing to */
template<class Filter, class Measurement, std::size_t Capacity>
    requires SequentialFilter<Filter, Measurement>
void apply_sequential(measurement_queue<Measurement, Capacity>& queue,
                      Filter& filter,
                      double callTime) {
    double timeOfLastMeasurement = queue.getTimeOfLastMeasurement();

    for (auto entry = queue.popEarliest(); entry.has_value(); entry = queue.popEarliest()) {
        auto& [timeTag, measurement] = entry.value();
        if (timeTag < timeOfLastMeasurement) continue;

        filter.timeUpdate(timeTag - timeOfLastMeasurement);
        filter.measurementUpdate(measurement);

        timeOfLastMeasurement = timeTag;
    }

    if (timeOfLastMeasurement < callTime) {
        filter.timeUpdate(callTime - timeOfLastMeasurement);
    }

    queue.setTimeOfLastMeasurement(timeOfLastMeasurement);
}

}  // namespace filtering

#endif
