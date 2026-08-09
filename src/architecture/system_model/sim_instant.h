// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMAheader_sim_instant
#define XMAheader_sim_instant

#include <compare>
#include <limits>
#include <stdint.h>

//! A simulation-time instant, comprising a nanosecond offset and an infinitesimal priority
/*!
 *  Unlike time in the real world (we think), simulated time proceeds in discrete,
 *  indivisible units. Furthermore, simulated time is hierarchical: two instants
 *  that occur the same number of nanoseconds after the start of the simulation
 *  may nonetheless be ordered by *priority*. This allows a coarse form of causal
 *  modeling: if two tasks occur at the same frequency and phase, but one depends
 *  on the other, we may ensure that the dependent occurs *after* its dependency
 *  by scheduling them at distinct instants differing only in priority.
 */
struct SimInstant final {
public:
    //! A nanosecond offset identifying a strictly temporal instant
    uint64_t realNanos = 0;

    //! A priority index dictating causal order *within* a temporal instant
    int64_t causalPriority = 0;

public:
    //! Obtain a simulation instant at the given nanosecond offset and at priority 0
    static SimInstant atNanos(uint64_t const realNanos) {
        return {.realNanos = realNanos, .causalPriority = 0};
    }

    //! Obtain a simulation instant at the current nanosecond offset and the given priority
    SimInstant atPriority(int64_t const causalPriority) const {
        return {.realNanos = this->realNanos, .causalPriority = causalPriority};
    }

    //! Obtain the latest possible simulation instant
    static SimInstant endOfTime() {
        return {
            .realNanos = std::numeric_limits<uint64_t>::max(),
            .causalPriority = std::numeric_limits<int64_t>::min(),
        };
    }

    //! Perform a three-way ("trichotomous") comparison with another instant
    std::strong_ordering operator<=>(SimInstant const &other) const {
        return (this->realNanos < other.realNanos) ? std::strong_ordering::less
             : (this->realNanos > other.realNanos) ? std::strong_ordering::greater
             // Priority order is inverted: priority 10 occurs *before* priority 5.
             : (this->causalPriority > other.causalPriority) ? std::strong_ordering::less
             : (this->causalPriority < other.causalPriority) ? std::strong_ordering::greater
                                                             : std::strong_ordering::equal;
    }

    //! Compare two instants for equality
    bool operator==(SimInstant const &other) const = default;
};

#endif
