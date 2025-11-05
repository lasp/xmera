// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "kalmanFilter.h"

/*!- Order the measurements chronologically (standard sort)
 @return void
 */
static void orderMeasurementsChronologically(xmera::MeasurementVector measurements) {
    std::sort(measurements.begin(),
              measurements.end(),
              [](std::optional<MeasurementModel> meas1, std::optional<MeasurementModel> meas2) {
                  if (!meas1.has_value()) return false;
                  if (!meas2.has_value()) return true;
                  return meas1.value().getTimeTag() < meas2.value().getTimeTag();
              });
}

/*! Take the relative position measurements and output an estimate of the
 spacecraft states in the inertial frame.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
extern void xmera::updateKalmanFilter(
    KalmanFilter& filterState,
    MeasurementVector measurements,
    uint64_t previousFilterTimeTag,
    uint64_t nextFilterTimeTag
) {
    /*! sort the measurment vector in chronological order */
    orderMeasurementsChronologically(measurements);

    double currentFilterTimeTag = previousFilterTimeTag;

    /*! Loop through all of the measurements assuming they are in chronological order by first testing if a value
     * has been populated in the measurements array*/
    for (auto& measurementElt : measurements) {
        if (!measurementElt.has_value()) continue;
        auto& measurement = measurementElt.value();

        /*! - If the time tag from a valid measurement is new compared to previous step,
        propagate and update the filter*/
        if (measurement.getTimeTag() < currentFilterTimeTag || !measurement.getValidity()) continue;

        /*! - time update to the measurement time and compute pre-fit residuals*/
        filterState.timeUpdate(measurement.getTimeTag() - currentFilterTimeTag);
        currentFilterTimeTag = measurement.getTimeTag();

        measurement.setPreFitResiduals(filterState.computeResiduals(measurement));

        /*! - measurement update and compute post-fit residuals  */
        auto computedMeasurement = filterState.measurementUpdate(measurement);

        measurement.setPostFitResiduals(
            measurement.subMeasurements(measurement.getObservation(), computedMeasurement));
    }

    /*! - If current clock time is further ahead than the last measurement time, then
    propagate to this current time-step*/
    if (nextFilterTimeTag > currentFilterTimeTag) {
        filterState.timeUpdate(nextFilterTimeTag - currentFilterTimeTag);
    }
}
