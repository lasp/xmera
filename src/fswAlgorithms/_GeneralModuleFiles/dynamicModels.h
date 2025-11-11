// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTER_DYN_MODELS_H
#define FILTER_DYN_MODELS_H

#include <fswAlgorithms/_GeneralModuleFiles/stateModels.h>

/*! @brief Measurement models used to map a state vector to a measurement */
using DynamicsModel = std::function<FilterStateVector(double time, FilterStateVector const& state)>;

namespace xmera {
    FilterStateVector propagate(
        DynamicsModel const& propagator,
        FilterStateVector state,
        std::array<double, 2> interval,
        double dt
    );
}

#endif
