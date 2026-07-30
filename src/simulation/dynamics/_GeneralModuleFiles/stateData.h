// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef STATE_DATA_H
#define STATE_DATA_H
#include <stdint.h>

#include <Eigen/Dense>

/*! @brief state data class*/
class StateData {
public:
    Eigen::MatrixXd state;       //!< [-] State value storage
    Eigen::MatrixXd stateDeriv;  //!< [-] State derivative value storage

public:
    StateData() = default;
    StateData(StateData const &inState) = default;        //!< class method
    explicit StateData(Eigen::MatrixXd const &newState);  //!< class method
    ~StateData() = default;
    void setState(Eigen::MatrixXd const &newState);       //!< class method
    void propagateState(double dt);                       //!< class method
    void setDerivative(Eigen::MatrixXd const &newDeriv);  //!< class method

    Eigen::MatrixXd const &getState() const {
        return state;
    }  //!< class method

    Eigen::MatrixXd const &getStateDeriv() const {
        return stateDeriv;
    }  //!< class method

    uint32_t getRowSize() const {
        return ((uint32_t) state.innerSize());
    }  //!< class method

    uint32_t getColumnSize() const {
        return ((uint32_t) state.outerSize());
    }  //!< class method

    void scaleState(double scaleFactor);  //!< class method

    StateData operator+(StateData const &operand) const;  //!< class method
    StateData operator*(double scaleFactor) const;        //!< class method
};

#endif /* STATE_DATA_H */
