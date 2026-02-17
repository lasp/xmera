// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "stateVecIntegrator.h"
#include "dynamicObject.h"

/*! @brief Constructor */
StateVecIntegrator::StateVecIntegrator(DynamicObject* dyn) { this->dynPtrs.push_back(dyn); }

/*! @brief Destructor */
StateVecIntegrator::~StateVecIntegrator(void) { this->dynPtrs.clear(); }
