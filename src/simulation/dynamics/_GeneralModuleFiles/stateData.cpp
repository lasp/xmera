// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "stateData.h"

StateData::StateData(const Eigen::MatrixXd& newState) :
    state(newState),
    stateDeriv(state) {
    stateDeriv.setZero();
}

void StateData::setState(const Eigen::MatrixXd& newState) {
    state = newState;
}

void StateData::propagateState(const double dt) { state += stateDeriv * dt; }

void StateData::setDerivative(const Eigen::MatrixXd& newDeriv) { stateDeriv = newDeriv; }

void StateData::scaleState(const double scaleFactor) { state *= scaleFactor; }

StateData StateData::operator+(const StateData& operand) const {
    return StateData(state + operand.getState());
}

StateData StateData::operator*(const double scaleFactor) const {
    StateData newState(state);
    newState.state *= scaleFactor;
    return newState;
}
