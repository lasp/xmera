#include "architecture/utilities/signalProcessing.h"

/**
 * Process a measurement into the low pass filter
 * @param measurement Eigen::Vector3d
 * @return void
 */
void LowPassFilter::processMeasurement(const Eigen::Vector3d& measurement){

    double omegaStep = this->filterStep*this->filterCutOff;
    this->currentState = 1/(2+omegaStep)*(this->currentState*(2-omegaStep) + omegaStep*(measurement+this->currentMeasurement));
    this->currentMeasurement = measurement;
};

/**
 * Get the filter current state
 * @return measurement Eigen::Vector3d
 */
Eigen::Vector3d LowPassFilter::getCurrentState() const {
    return this->currentState;
};

/**
 * Set the time step used in the low pass filter
 * @param filterStepSeconds double
 */
void LowPassFilter::setFilterStep(const double filterStepSeconds){
    this->filterStep = filterStepSeconds;
};

/**
 * Get the time step used in the low pass filter
 * @return filterStepSeconds double
 */
double LowPassFilter::getFilterStep() const{
    return this->filterStep;
};

/**
 * Set the cut off value (norm of the measurements) in the low pass
 * @param cutOffValue double
 */
void LowPassFilter::setFilterCutoff(const double cutOffValue){
    this->filterCutOff = cutOffValue;
};

/**
 * Get the cut off value (norm of the measurements) in the low pass
 * @return cutOffValue double
 */
double LowPassFilter::getFilterCutoff() const{
    return this->filterCutOff;
};
