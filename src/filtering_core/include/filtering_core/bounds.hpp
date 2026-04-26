// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_BOUNDS_HPP
#define FILTERING_CORE_BOUNDS_HPP

// Compile-time bounds for filtering algorithms.
//
// Per-filter bounds (state size, measurement vector size, queue capacity)
// belong on the filter's Spec/Algorithm — not here. This header is reserved
// for cross-filter conventions that genuinely do need a single source of
// truth, if they arise.
//
// The MAX_MEASUREMENT_NUMBER / MAX_MEASUREMENT_VECTOR / MAX_STATES_VECTOR
// constants from filterInterfaceDefinitions.h are intentionally NOT carried
// over: the new design has each filter declare its own bounds via
// `measurement_queue<MeasurementType, N>`, `Spec::State::size`, and
// `Measurement::size`.

namespace filtering {
}

#endif
