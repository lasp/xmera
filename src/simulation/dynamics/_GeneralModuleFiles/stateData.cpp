// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "stateData.h"

StateData::StateData(Eigen::MatrixXd const &newState) : state(newState), stateDeriv(state) {
    stateDeriv.setZero();
}

void StateData::setState(Eigen::MatrixXd const &newState) {
    state = newState;
}

void StateData::propagateState(double const dt) {
    state += stateDeriv * dt;
}

void StateData::setDerivative(Eigen::MatrixXd const &newDeriv) {
    stateDeriv = newDeriv;
}

void StateData::scaleState(double const scaleFactor) {
    state *= scaleFactor;
}

StateData StateData::operator+(StateData const &operand) const {
    return StateData(state + operand.getState());
}

StateData StateData::operator*(double const scaleFactor) const {
    StateData newState(state);
    newState.state *= scaleFactor;
    return newState;
}
