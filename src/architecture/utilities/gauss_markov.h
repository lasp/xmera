// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _GaussMarkov_HH_
#define _GaussMarkov_HH_

#include <stdint.h>

#include <cmath>
#include <Eigen/Dense>
#include <random>

/*! @brief This module is used to apply a second-order bounded Gauss-Markov random walk
    on top of an upper level process.  The intent is that the caller will perform
    the set methods (setUpperBounds, setNoiseMatrix, setPropMatrix) as often as
    they need to, call computeNextState, and then call getCurrentState cyclically.

    The number of states N is a compile-time template parameter, so all storage is
    fixed-size and dimension mismatches are caught at compile time.
*/
template<int N>
class GaussMarkov {
    static_assert(N > 0, "GaussMarkov requires a positive number of states");

public:
    using StateVector = Eigen::Matrix<double, N, 1>;  //!< -- Fixed-size state/bounds vector type
    using StateMatrix = Eigen::Matrix<double, N, N>;  //!< -- Fixed-size propagation/noise matrix type

    /*! The default constructor initializes the random number generator used for the walks. */
    GaussMarkov() {
        this->RNGSeed = 0x1bad'cad1;
        std::normal_distribution<double>::param_type updatePair(0.0, 1.0 / 3.0);
        this->rGen.seed((unsigned int) this->RNGSeed);
        this->rNum.param(updatePair);
    }

    /*! Seeded constructor.
        @param newSeed The seed to use in the random number generator */
    explicit GaussMarkov(uint64_t newSeed) : GaussMarkov() {
        this->setRNGSeed(newSeed);
    }

    /*! This method performs almost all of the work for the Gauss Markov random walk.  It uses the
        current random walk configuration, propagates the current state, and then applies appropriate
        errors to the states to set the current error level.
        @return void */
    void computeNextState() {
        //! - Propagate the state forward in time using the propMatrix and the currentState
        StateVector errorVector = this->currentState;
        this->currentState = this->propMatrix * errorVector;

        //! - Compute the random numbers used for each state.  Note that the same generator is used for all
        StateVector ranNums;
        for (int i = 0; i < N; i++) {
            ranNums[i] = this->rNum(this->rGen);
            if (this->stateBounds[i] > 0.0) {
                double stateCalc = std::fabs(this->currentState[i]) > this->stateBounds[i] * 1E-10
                                     ? std::fabs(this->currentState[i])
                                     : this->stateBounds[i];

                double boundCheck = (this->stateBounds[i] * 2.0 - stateCalc) / stateCalc;
                boundCheck = boundCheck > this->stateBounds[i] * 1E-10 ? boundCheck : this->stateBounds[i] * 1E-10;
                boundCheck = 1.0 / std::exp(boundCheck * boundCheck * boundCheck);
                boundCheck *= std::copysign(boundCheck, -this->currentState[i]);
                ranNums[i] += boundCheck;
            }
        }

        //! - Apply the noise matrix to the random numbers to get error values
        errorVector = this->noiseMatrix * ranNums;

        //! - Add the new errors to the currentState to get a good currentState
        this->currentState += errorVector;
    }

    /*!@brief Method does just what it says, seeds the random number generator
       @param newSeed The seed to use in the random number generator
       @return void*/
    void setRNGSeed(uint64_t newSeed) {
        this->rGen.seed((unsigned int) newSeed);
        this->RNGSeed = newSeed;
    }

    /*!@brief Method returns the current random walk state from model
       @return The private currentState which is the vector of random walk values*/
    StateVector const &getCurrentState() const {
        return this->currentState;
    }

    /*!@brief Set the upper bounds on the random walk to newBounds
       @param newBounds the bounds to put on the random walk states
       @return void*/
    void setUpperBounds(StateVector const &newBounds) {
        this->stateBounds = newBounds;
    }

    /*!@brief Set the noiseMatrix that is used to define error sigmas
       @param noise The new value to use for the noiseMatrix variable (error sigmas)
       @return void*/
    void setNoiseMatrix(StateMatrix const &noise) {
        this->noiseMatrix = noise;
    }

    /*!@brief Set the propagation matrix that is used to propagate the state.
       @param prop The new value for the state propagation matrix
       @return void*/
    void setPropMatrix(StateMatrix const &prop) {
        this->propMatrix = prop;
    }

private:
    StateVector stateBounds = StateVector::Zero();   //!< -- Upper bounds to use for markov
    StateVector currentState = StateVector::Zero();  //!< -- State of the markov model
    StateMatrix propMatrix = StateMatrix::Zero();    //!< -- Matrix to propagate error state with
    StateMatrix noiseMatrix =
        StateMatrix::Zero();  //!< -- Cholesky-decomposition or matrix square root of the covariance matrix
    uint64_t RNGSeed;         //!< -- Seed for random number generator
    std::minstd_rand rGen;    //!< -- Random number generator for model
    std::normal_distribution<double> rNum;  //!< -- Random number distribution for model
};

#endif /* _GaussMarkov_HH_ */
