// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef STATE_DATA_H
#define STATE_DATA_H
#include <Eigen/Dense>
#include <stdint.h>

/*! @brief state data class*/
class StateData {
   public:
    Eigen::MatrixXd state;       //!< [-] State value storage
    Eigen::MatrixXd stateDeriv;  //!< [-] State derivative value storage

   public:
    StateData() = default;
    StateData(const StateData& inState) = default;                            //!< class method
    explicit StateData(const Eigen::MatrixXd& newState);                      //!< class method
    ~StateData() = default;
    void setState(const Eigen::MatrixXd& newState);                           //!< class method
    void propagateState(double dt);                                           //!< class method
    void setDerivative(const Eigen::MatrixXd& newDeriv);                      //!< class method
    Eigen::MatrixXd const &getState() const { return state; }                        //!< class method
    Eigen::MatrixXd const &getStateDeriv() const { return stateDeriv; }              //!< class method
    uint32_t getRowSize() const { return ((uint32_t)state.innerSize()); }     //!< class method
    uint32_t getColumnSize() const { return ((uint32_t)state.outerSize()); }  //!< class method
    void scaleState(double scaleFactor);                                      //!< class method

    StateData operator+(const StateData& operand) const;  //!< class method
    StateData operator*(double scaleFactor) const;        //!< class method
};

#endif /* STATE_DATA_H */
