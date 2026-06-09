// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_MEASUREMENT_QUEUE_HPP
#define FILTERING_CORE_MEASUREMENT_QUEUE_HPP

#include <array>
#include <optional>
#include <utility>

namespace filtering {

/*! Measurement container.  */
template<typename Measurement, std::size_t CAPACITY>
class measurement_queue final {
public:
    bool isEmpty() const { return this->size == 0; }
    bool isFull()  const { return this->size >= CAPACITY; }

    /*! Set the time of last measurement.
     *  @param t [s] sim time of the last drained measurement */
    void   setTimeOfLastMeasurement(double t)   { this->lastMeasurementTime = t; }
    /*! @return time of the most recent measurement processed */
    double getTimeOfLastMeasurement() const     { return this->lastMeasurementTime; }


    /*! Insertion-sort a measurement into the queue by descending timeTag.
     *  @return true if enqueued, false if the queue was full
     *  @param timeTag     [s] time the measurement was taken
     *  @param measurement [-] measurement payload (moved in) */
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

    /*! Drop all measurements and reset the time anchor to zero. */
    void clear() {
        while (this->size > 0) {
            this->size -= 1;
            this->measurements[this->size] = std::nullopt;
        }
        this->lastMeasurementTime = 0;
    }

    /*! Remove and return the earliest queued measurement.
     *  @return std::optional<{timeTag, measurement}>; nullopt if empty */
    std::optional<std::pair<double, Measurement>> popEarliest() {
        if (this->isEmpty()) return std::nullopt;

        this->size -= 1;
        return std::exchange(this->measurements[this->size], std::nullopt);
    }

private:
    std::size_t                                                         size = 0;
    std::array<std::optional<std::pair<double, Measurement>>, CAPACITY> measurements = {};
    double                                                              lastMeasurementTime = 0;
};

}  // namespace filtering

#endif
