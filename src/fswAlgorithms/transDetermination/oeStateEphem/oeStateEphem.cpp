/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

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
void OEStateEphem::updateState(const uint64_t callTime) {
    auto tmpOutputState = this->algorithm.updateState(callTime);
    this->stateFitOutMsg.write(&tmpOutputState, moduleID, callTime);
}

void OEStateEphem::setCentralBodyGravitationalParameter(const double mu) {
    this->algorithm.setCentralBodyGravitationalParameter(mu);
};

double OEStateEphem::getCentralBodyGravitationalParameter() const {
    return this->algorithm.getCentralBodyGravitationalParameter();
};

void OEStateEphem::setArcNumberOfCoefficients(const unsigned int arcNumber, const unsigned int numberOfCoefficients) {
    this->algorithm.setArcNumberOfCoefficients(arcNumber, numberOfCoefficients);
};

unsigned int OEStateEphem::getArcNumberOfCoefficients(const unsigned int arcNumber) const {
    return this->algorithm.getArcNumberOfCoefficients(arcNumber);
};

void OEStateEphem::setArcMiddleTime(const unsigned int arcNumber, const double timeMiddle) {
    this->algorithm.setArcMiddleTime(arcNumber, timeMiddle);
};

double OEStateEphem::getArcMiddleTime(const unsigned int arcNumber) const {
    return this->algorithm.getArcMiddleTime(arcNumber);
};

void OEStateEphem::setArcRadiusTime(const unsigned int arcNumber, const double timeRadius) {
    this->algorithm.setArcRadiusTime(arcNumber, timeRadius);
};

double OEStateEphem::getArcRadiusTime(const unsigned int arcNumber) const {
    return this->algorithm.getArcRadiusTime(arcNumber);
};

void OEStateEphem::setArcAnomalyFlag(const unsigned int arcNumber, const unsigned int anomalyFlag) {
    this->algorithm.setArcAnomalyFlag(arcNumber, anomalyFlag);
};

unsigned int OEStateEphem::getArcAnomalyFlag(unsigned int arcNumber) const {
    return this->algorithm.getArcAnomalyFlag(arcNumber);
};

void OEStateEphem::setArcRadiusPeriapsisCoefficients(
    const unsigned int arcNumber,
    const std::array<double, MAX_OE_COEFF>& radiusPeriapsisCoefficients) {
    this->algorithm.setArcRadiusPeriapsisCoefficients(arcNumber, radiusPeriapsisCoefficients);
};

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcRadiusPeriapsisCoefficients(const unsigned int arcNumber) {
    return this->algorithm.getArcRadiusPeriapsisCoefficients(arcNumber);
};

void OEStateEphem::setArcEccentricityCoefficients(const unsigned int arcNumber,
                                                  const std::array<double, MAX_OE_COEFF>& eccentricityCoefficients) {
    this->algorithm.setArcEccentricityCoefficients(arcNumber, eccentricityCoefficients);
};

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcEccentricityCoefficients(const unsigned int arcNumber) {
    return this->algorithm.getArcEccentricityCoefficients(arcNumber);
};

void OEStateEphem::setArcInclinationCoefficients(const unsigned int arcNumber,
                                                 const std::array<double, MAX_OE_COEFF>& inclinationCoefficients) {
    this->algorithm.setArcInclinationCoefficients(arcNumber, inclinationCoefficients);
};

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcInclinationCoefficients(const unsigned int arcNumber) {
    return this->algorithm.getArcInclinationCoefficients(arcNumber);
};

void OEStateEphem::setArcArgPeriapsisCoefficients(const unsigned int arcNumber,
                                                  const std::array<double, MAX_OE_COEFF>& argPeriapsisCoefficients) {
    this->algorithm.setArcArgPeriapsisCoefficients(arcNumber, argPeriapsisCoefficients);
};

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcArgPeriapsisCoefficients(const unsigned int arcNumber) {
    return this->algorithm.getArcArgPeriapsisCoefficients(arcNumber);
};

void OEStateEphem::setArcRaanCoefficients(const unsigned int arcNumber,
                                          const std::array<double, MAX_OE_COEFF>& raanCoefficients) {
    this->algorithm.setArcRaanCoefficients(arcNumber, raanCoefficients);
};

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcRaanCoefficients(const unsigned int arcNumber) {
    return this->algorithm.getArcRaanCoefficients(arcNumber);
};

void OEStateEphem::setArcTrueAnomalyCoefficients(const unsigned int arcNumber,
                                                 const std::array<double, MAX_OE_COEFF>& trueAnomalyCoefficients) {
    this->algorithm.setArcTrueAnomalyCoefficients(arcNumber, trueAnomalyCoefficients);
};

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcTrueAnomalyCoefficients(const unsigned int arcNumber) {
    return this->algorithm.getArcTrueAnomalyCoefficients(arcNumber);
};
