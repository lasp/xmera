// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_MEASUREMENT_QUEUE_HPP
#define FILTERING_CORE_MEASUREMENT_QUEUE_HPP

#include <array>
#include <optional>
#include <utility>

namespace filtering {

// Bounded-capacity, time-ordered container of measurements.
//
// Pure container: enqueue, popEarliest, clear, isFull, isEmpty. Scheduling
// (how a filter consumes the queue over an update window) is intentionally
// not a member here — different filter families want different scheduling
// policies. The canonical sequential-Kalman policy is the free template
// `apply_sequential` defined below; other styles (batch, iterated,
// out-of-order rewind, ...) are added as additional free functions without
// touching this container.
//
// `Measurement` may be a single kind (e.g. HeadingMeasurement) or a small
// closed tagged union (enum tag + union + visit) for a filter that consumes
// heterogeneous measurements on one timeline — a freestanding stand-in for
// std::variant. The queue is kind-agnostic either way; a multi-sensor filter
// visits the union in its measurementUpdate. See
// filtering_core/_tests/test_heterogeneous_measurements.cpp.
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
