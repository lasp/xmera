// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "saturate.h"
#include <iostream>
#include <math.h>

/*! The constructor initialies the random number generator used for the walks*/
Saturate::Saturate() {}

Saturate::Saturate(int64_t size) : Saturate() {
    this->numStates = size;
    this->stateBounds.resize(numStates, 2);
}
/*! The destructor is a placeholder for one that might do something*/
Saturate::~Saturate() {}

/*!
    @brief This method should be used as the standard way to saturate an output. It will also be utilized by
other utilities
    @param unsaturatedStates a vector of the unsaturated states
    @return saturatedStates
 */
Eigen::VectorXd Saturate::saturate(Eigen::VectorXd unsaturatedStates) {
    Eigen::VectorXd workingStates;
    workingStates.resize(this->numStates);
    for (int64_t i = 0; i < this->numStates; i++) {
        workingStates[(int)i] = std::min(unsaturatedStates[i], this->stateBounds(i, 1));
        workingStates[(int)i] = std::max(workingStates[i], this->stateBounds(i, 0));
    }
    return workingStates;
}

/*!
    @brief sets upper and lower bounds for each state
    @param bounds one row for each state. lower bounds in left column, upper in right column
    @return void
 */
void Saturate::setBounds(Eigen::MatrixXd bounds) { this->stateBounds = bounds; }
