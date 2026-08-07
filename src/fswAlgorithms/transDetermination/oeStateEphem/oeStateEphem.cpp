// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "oeStateEphem.h"

/*!
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void OEStateEphem::reset(uint64_t callTime) {
    if (!this->clockCorrInMsg.isLinked()) {
        throw std::invalid_argument("OEStateEphem.clockCorrInMsg wasn't connected.");
    }
    this->algorithm.reset(callTime, this->clockCorrInMsg());
}

/*! This method takes the current time and computes the state of the object
    using that time and the stored Chebyshev coefficients.  If the time provided
    is outside the specified range, the position vectors rail high/low appropriately.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void OEStateEphem::updateState(uint64_t const callTime) {
    auto tmpOutputState = this->algorithm.updateState(callTime);
    this->stateFitOutMsg.write(tmpOutputState, moduleID, callTime);
}

void OEStateEphem::setCentralBodyGravitationalParameter(double const mu) {
    this->algorithm.setCentralBodyGravitationalParameter(mu);
}

double OEStateEphem::getCentralBodyGravitationalParameter() const {
    return this->algorithm.getCentralBodyGravitationalParameter();
}

void OEStateEphem::setArcNumberOfCoefficients(unsigned int const arcNumber, unsigned int const numberOfCoefficients) {
    this->algorithm.setArcNumberOfCoefficients(arcNumber, numberOfCoefficients);
}

unsigned int OEStateEphem::getArcNumberOfCoefficients(unsigned int const arcNumber) const {
    return this->algorithm.getArcNumberOfCoefficients(arcNumber);
}

void OEStateEphem::setArcMiddleTime(unsigned int const arcNumber, double const timeMiddle) {
    this->algorithm.setArcMiddleTime(arcNumber, timeMiddle);
}

double OEStateEphem::getArcMiddleTime(unsigned int const arcNumber) const {
    return this->algorithm.getArcMiddleTime(arcNumber);
}

void OEStateEphem::setArcRadiusTime(unsigned int const arcNumber, double const timeRadius) {
    this->algorithm.setArcRadiusTime(arcNumber, timeRadius);
}

double OEStateEphem::getArcRadiusTime(unsigned int const arcNumber) const {
    return this->algorithm.getArcRadiusTime(arcNumber);
}

void OEStateEphem::setArcAnomalyFlag(unsigned int const arcNumber, unsigned int const anomalyFlag) {
    this->algorithm.setArcAnomalyFlag(arcNumber, anomalyFlag);
}

unsigned int OEStateEphem::getArcAnomalyFlag(unsigned int arcNumber) const {
    return this->algorithm.getArcAnomalyFlag(arcNumber);
}

void OEStateEphem::setArcRadiusPeriapsisCoefficients(
    unsigned int const arcNumber,
    std::array<double, MAX_OE_COEFF> const &radiusPeriapsisCoefficients
) {
    this->algorithm.setArcRadiusPeriapsisCoefficients(arcNumber, radiusPeriapsisCoefficients);
}

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcRadiusPeriapsisCoefficients(unsigned int const arcNumber) {
    return this->algorithm.getArcRadiusPeriapsisCoefficients(arcNumber);
}

void OEStateEphem::setArcEccentricityCoefficients(
    unsigned int const arcNumber,
    std::array<double, MAX_OE_COEFF> const &eccentricityCoefficients
) {
    this->algorithm.setArcEccentricityCoefficients(arcNumber, eccentricityCoefficients);
}

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcEccentricityCoefficients(unsigned int const arcNumber) {
    return this->algorithm.getArcEccentricityCoefficients(arcNumber);
}

void OEStateEphem::setArcInclinationCoefficients(
    unsigned int const arcNumber,
    std::array<double, MAX_OE_COEFF> const &inclinationCoefficients
) {
    this->algorithm.setArcInclinationCoefficients(arcNumber, inclinationCoefficients);
}

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcInclinationCoefficients(unsigned int const arcNumber) {
    return this->algorithm.getArcInclinationCoefficients(arcNumber);
}

void OEStateEphem::setArcArgPeriapsisCoefficients(
    unsigned int const arcNumber,
    std::array<double, MAX_OE_COEFF> const &argPeriapsisCoefficients
) {
    this->algorithm.setArcArgPeriapsisCoefficients(arcNumber, argPeriapsisCoefficients);
}

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcArgPeriapsisCoefficients(unsigned int const arcNumber) {
    return this->algorithm.getArcArgPeriapsisCoefficients(arcNumber);
}

void OEStateEphem::setArcRaanCoefficients(
    unsigned int const arcNumber,
    std::array<double, MAX_OE_COEFF> const &raanCoefficients
) {
    this->algorithm.setArcRaanCoefficients(arcNumber, raanCoefficients);
}

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcRaanCoefficients(unsigned int const arcNumber) {
    return this->algorithm.getArcRaanCoefficients(arcNumber);
}

void OEStateEphem::setArcTrueAnomalyCoefficients(
    unsigned int const arcNumber,
    std::array<double, MAX_OE_COEFF> const &trueAnomalyCoefficients
) {
    this->algorithm.setArcTrueAnomalyCoefficients(arcNumber, trueAnomalyCoefficients);
}

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcTrueAnomalyCoefficients(unsigned int const arcNumber) {
    return this->algorithm.getArcTrueAnomalyCoefficients(arcNumber);
}
