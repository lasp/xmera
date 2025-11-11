// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "stateModels.h"

/*! Set the positional components of your state (cartesian position, attitude, etc)
   @param Eigen::VectorXd positionComponents
*/
Eigen::Index FilterStateVector::size() const {
    Eigen::Index totalSize = 0;
    if (this->hasPosition()) {
        totalSize += this->getPositionStates().size();
    }
    if (this->hasVelocity()) {
        totalSize += this->getVelocityStates().size();
    }
    if (this->hasAcceleration()) {
        totalSize += this->getAccelerationStates().size();
    }
    if (this->hasBias()) {
        totalSize += this->getBiasStates().size();
    }
    if (this->hasConsider()) {
        totalSize += this->getConsiderStates().size();
    }
    return totalSize;
}

/*! Add a eigen vector to a state, assuming the order of position, velocity, acceleration, bias, consider
   @param Eigen::VectorXd vector
   @return FilterStateVector sumVector
*/
FilterStateVector FilterStateVector::addVector(const Eigen::VectorXd& vector) const {
    assert(vector.size() == this->size());
    Eigen::Index lastIndex = 0;
    FilterStateVector sum;
    if (this->hasPosition()) {
        sum.position = PositionState<Eigen::Dynamic> {
              this->getPositionStates()
            + vector.segment(lastIndex, this->getPositionStates().size())
        };
        lastIndex += this->getPositionStates().size();
    }
    if (this->hasVelocity()) {
        sum.velocity = VelocityState<Eigen::Dynamic> {
              this->getVelocityStates()
            + vector.segment(lastIndex, this->getVelocityStates().size())
        };
        lastIndex += this->getVelocityStates().size();
    }
    if (this->hasAcceleration()) {
        sum.acceleration = AccelerationState<Eigen::Dynamic> {
              this->getAccelerationStates()
            + vector.segment(lastIndex, this->getAccelerationStates().size())
        };
        lastIndex += this->getAccelerationStates().size();
    }
    if (this->hasBias()) {
        sum.bias = BiasState<Eigen::Dynamic> {
              this->getBiasStates()
            + vector.segment(lastIndex, this->getBiasStates().size())
        };
        lastIndex += this->getBiasStates().size();
    }
    if (this->hasConsider()) {
        sum.considerParameters = ConsiderState<Eigen::Dynamic> {
              this->getConsiderStates()
            + vector.segment(lastIndex, this->getConsiderStates().size())
        };
        lastIndex += this->getConsiderStates().size();
    }
    return sum;
}

/*! Add two states together
   @param FilterStateVector vector
   @return FilterStateVector sumVector
*/
FilterStateVector FilterStateVector::add(const FilterStateVector& vector) const {
    FilterStateVector sum;
    if (this->hasPosition() && vector.hasPosition()) {
        sum.position = PositionState<Eigen::Dynamic> {
              this->getPositionStates()
            + vector.getPositionStates()
        };
    }
    if (this->hasVelocity() && vector.hasVelocity()) {
        sum.velocity = VelocityState<Eigen::Dynamic> {
              this->getVelocityStates()
            + vector.getVelocityStates()
        };
    }
    if (this->hasAcceleration() && vector.hasAcceleration()) {
        sum.acceleration = AccelerationState<Eigen::Dynamic> {
              this->getAccelerationStates()
            + vector.getAccelerationStates()
        };
    }
    if (this->hasBias() && vector.hasBias()) {
        sum.bias = BiasState<Eigen::Dynamic> {
              this->getBiasStates()
            + vector.getBiasStates()
        };
    }
    if (this->hasConsider() && vector.hasConsider()) {
        sum.considerParameters = ConsiderState<Eigen::Dynamic> {
              this->getConsiderStates()
            + vector.getConsiderStates()
        };
    }
    sum.attachStm(this->detachStm() + vector.detachStm());
    return sum;
}

/*! Scale a state vector by a constant
   @param double scalar
   @return FilterStateVector scaledVector
*/
FilterStateVector FilterStateVector::scale(const double scalar) const {
    FilterStateVector scaledVector;
    if (this->hasPosition()) {
        scaledVector.position = PositionState<Eigen::Dynamic> {
            this->getPositionStates() * scalar
        };
    }
    if (this->hasVelocity()) {
        scaledVector.velocity = VelocityState<Eigen::Dynamic> {
            this->getVelocityStates() * scalar
        };
    }
    if (this->hasAcceleration()) {
        scaledVector.acceleration = AccelerationState<Eigen::Dynamic> {
            this->getAccelerationStates() * scalar
        };
    }
    if (this->hasBias()) {
        scaledVector.bias = BiasState<Eigen::Dynamic> {
            this->getBiasStates() * scalar
        };
    }
    if (this->hasConsider()) {
        scaledVector.considerParameters = ConsiderState<Eigen::Dynamic> {
            this->getConsiderStates() * scalar
        };
    }
    scaledVector.attachStm(this->detachStm() * scalar);
    return scaledVector;
}

/*! Return the full state vector
   @return Eigen::VectorXd fullFilterStateVector
*/
Eigen::VectorXd FilterStateVector::returnValues() const {
    Eigen::VectorXd stateVectorValues(this->size());
    Eigen::Index lastIndex = 0;
    if (this->hasPosition()) {
        stateVectorValues.segment(lastIndex, this->getPositionStates().size())
            = this->getPositionStates();
        lastIndex += this->getPositionStates().size();
    }
    if (this->hasVelocity()) {
        stateVectorValues.segment(lastIndex, this->getVelocityStates().size())
            = this->getVelocityStates();
        lastIndex += this->getVelocityStates().size();
    }
    if (this->hasAcceleration()) {
        stateVectorValues.segment(lastIndex, this->getAccelerationStates().size())
            = this->getAccelerationStates();
        lastIndex += this->getAccelerationStates().size();
    }
    if (this->hasBias()) {
        stateVectorValues.segment(lastIndex, this->getBiasStates().size())
            = this->getBiasStates();
        lastIndex += this->getBiasStates().size();
    }
    if (this->hasConsider()) {
        stateVectorValues.segment(lastIndex, this->getConsiderStates().size())
            = this->getConsiderStates();
        lastIndex += this->getBiasStates().size();
    }
    return stateVectorValues;
}

/*! Check if the state vector has a position state
   @return bool
*/
bool FilterStateVector::hasPosition() const {
    return this->position.has_value();
}

/*! Set the positional components of your state (cartesian position, attitude, etc)
   @param Eigen::VectorXd positionComponents
*/
void FilterStateVector::setPosition(const PositionState<Eigen::Dynamic>& positionState) {
    this->position = positionState;
}

/*! Get the positional components of your state (cartesian position, attitude, etc)
   @return Eigen::VectorXd
*/
Eigen::VectorXd FilterStateVector::getPositionStates() const {
    return this->position.value().getValues();
}

/*! Check if the state vector has a velocity state
   @return bool
*/
bool FilterStateVector::hasVelocity() const {
    return this->velocity.has_value();
}

/*! Set the velocity components of your state (cartesian velocity, angular rate, etc)
   @param Eigen::VectorXd velocityComponents
*/
void FilterStateVector::setVelocity(const VelocityState<Eigen::Dynamic>& velocityState) {
    this->velocity = velocityState;
}

/*! Get the velocity components of your state (cartesian velocity, angular rate, etc)
   @return Eigen::VectorXd
*/
Eigen::VectorXd FilterStateVector::getVelocityStates() const {
    return this->velocity.value().getValues();
}

/*! Check if the state vector has a acceleration state
   @return bool
*/
bool FilterStateVector::hasAcceleration() const {
    return this->acceleration.has_value();
}

/*! Set the acceleration class of your state (cartesian acceleration, angular acceleration, etc)
   @param Eigen::VectorXd velocityComponents
*/
void FilterStateVector::setAcceleration(const AccelerationState<Eigen::Dynamic>& accelerationState) {
    this->acceleration = accelerationState;
}

/*! Get the acceleration components of your state (cartesian acceleration, angular acceleration, etc)
   @return Eigen::VectorXd
*/
Eigen::VectorXd FilterStateVector::getAccelerationStates() const {
    return this->acceleration.value().getValues();
}

/*! Check if the state vector has a bias state
   @return bool
*/
bool FilterStateVector::hasBias() const {
    return this->bias.has_value();
}

/*! Set the bias class of your state (cartesian bias, angular bias, etc)
   @param Eigen::VectorXd velocityComponents
*/
void FilterStateVector::setBias(const BiasState<Eigen::Dynamic>& biasState) {
    this->bias = biasState;
}

/*! Get the bias components of your state (cartesian bias, angular bias, etc)
   @return Eigen::VectorXd
*/
Eigen::VectorXd FilterStateVector::getBiasStates() const {
    return this->bias.value().getValues();
}

/*! Check if the state vector has a considerParameters state
   @return bool
*/
bool FilterStateVector::hasConsider() const {
    return this->considerParameters.has_value();
}

/*! Set the considerParameters class of your state (cartesian considerParameters, angular considerParameters, etc)
   @param Eigen::VectorXd velocityComponents
*/
void FilterStateVector::setConsider(const ConsiderState<Eigen::Dynamic>& considerParametersState) {
    this->considerParameters = considerParametersState;
}

/*! Get the considerParameters components of your state (cartesian considerParameters, angular considerParameters, etc)
   @return Eigen::VectorXd
*/
Eigen::VectorXd FilterStateVector::getConsiderStates() const {
    return this->considerParameters.value().getValues();
}

/*! Attach the state transition matrix of your state for simultaneous propagation
   @param Eigen::MatrixXd stm
*/
void FilterStateVector::attachStm(const Eigen::MatrixXd& stm) {
    this->stm = stm;
}

/*! Detach the state transition matrix of your state for simultaneous propagation
   @return Eigen::MatrixXd stm
*/
Eigen::MatrixXd FilterStateVector::detachStm() const {
    return this->stm;
}
