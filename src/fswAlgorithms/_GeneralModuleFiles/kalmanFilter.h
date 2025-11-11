// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef KALMAN_FILTER_INTERFACE_HPP
#define KALMAN_FILTER_INTERFACE_HPP

#include <optional>

/*! @brief Kalman Filter interface */
template<typename Measurement>
class KalmanFilter {
public:
    KalmanFilter() = default;
    virtual ~KalmanFilter() = default;

    virtual void reset() = 0;
    virtual void timeUpdate(double dt) = 0;
    virtual void measurementUpdate(Measurement& measurement) = 0;
};

namespace xmera {
    namespace detail {
        template<typename Base, typename Derived>
        concept base_of = std::derived_from<Derived, Base>;
    }

    template<typename Measurement, size_t CAPACITY>
    class measurement_queue final {
    public:
        bool isEmpty() {
            return (this->size <= 0);
        }

        bool isFull() {
            return (CAPACITY <= this->size);
        }

        bool enqueue(double timeTag, Measurement&& measurement) {
            if (this->isFull()) return false;

            size_t insertionIndex = this->size;
            while (0 < insertionIndex) {
                if (timeTag <= this->measurements[insertionIndex - 1].value().first) break;

                this->measurements[insertionIndex] =
                    std::move(this->measurements[insertionIndex - 1]);

                insertionIndex -= 1;
            }

            this->measurements[insertionIndex] = {timeTag, measurement};

            return true;
        }

        void clear() {
            while (0 < this->size) {
                this->size -= 1;
                this->measurements[this->size] = std::nullopt;
            }
        }

        std::optional<std::pair<double, Measurement>> popEarliest() {
            if (this->isEmpty()) return std::nullopt;

            this->size -= 1;
            return std::exchange(this->measurements[this->size], std::nullopt);
        }

        template<detail::base_of<Measurement> MeasurementBase>
        void applyToFilter(
            KalmanFilter<MeasurementBase>& filterState,
            double previousSimSeconds,
            double nextSimSeconds
        ) {
            double currentSimSeconds = previousSimSeconds;

            for (size_t i = this->size; 0 < i; i -= 1) {
                auto& measurement = this->measurements[i - 1].value();

                /*! - If the time tag from a valid measurement is new compared to previous step,
                propagate and update the filter*/
                if (measurement.first < currentSimSeconds) continue;

                /*! - time update to the measurement time */
                filterState.timeUpdate(measurement.first - currentSimSeconds);

                /*! - compute pre-fit residuals, measurement update, and compute post-fit residuals  */
                filterState.measurementUpdate(measurement.second);

                currentSimSeconds = measurement.first;
            }

            /*! - If current clock time is further ahead than the last measurement time, then
            propagate to this current time-step*/
            if (currentSimSeconds < nextSimSeconds) {
                filterState.timeUpdate(nextSimSeconds - currentSimSeconds);
            }
        }

    private:
        // INVARIANT: `measurement[i].has_value() == (i < size)`
        //   That is, all initialized values appear before all uninitialized values.
        // INVARIANT: `measurements[i + 1].value().first <= measurements[i].value().first`
        //   That is, bigger time tags are earlier in the list.
        size_t size = 0;
        std::array<std::optional<std::pair<double, Measurement>>, CAPACITY> measurements = {};
    };
}

#endif /* KALMAN_FILTER_INTERFACE_HPP */
