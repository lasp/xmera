/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric Space Physics, University of Colorado at Boulder

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

#include "oeStateEphemAlgorithm.h"
#include <architecture/utilities/eigenSupport.h>

void OEStateEphemAlgorithm::reset(uint64_t callTime, const TDBVehicleClockCorrelationMsgPayload& timePayload) {
    this->spacecraftTime = timePayload;
}

ChebyshevFitArc OEStateEphemAlgorithm::findCurrentArc(uint64_t callTime,
                                                      const TDBVehicleClockCorrelationMsgPayload& localTime) {
    /*! - compute time for fitting interval */
    this->currentEphTime = callTime * 1e-9 + localTime.ephemerisTime - localTime.vehicleClockTime;

    /*! - select the fitting coefficients for the nearest fit interval */
    uint32_t nearestArc = 0;
    double smallestTimeDifference = fabs(this->currentEphTime - this->fitCoefficients[0].ephemerisTimeMiddle);
    for (auto i = 1; i < MAX_OE_RECORDS; ++i) {
        double timeDifference = fabs(this->currentEphTime - this->fitCoefficients[i].ephemerisTimeMiddle);
        if (timeDifference < smallestTimeDifference) {
            nearestArc = i;
            smallestTimeDifference = timeDifference;
        }
    }

    /*! - determine the scaled fitting time */
    return this->fitCoefficients[nearestArc];
}

double OEStateEphemAlgorithm::scaleEphemerisTime(const ChebyshevFitArc& arc) const {
    double currentScaledValue = (this->currentEphTime - arc.ephemerisTimeMiddle) / arc.ephemerisTimeRadius;
    if (fabs(currentScaledValue) > 1.0) {
        currentScaledValue = currentScaledValue / fabs(currentScaledValue);
    }
    return currentScaledValue;
}

ClassicalElements OEStateEphemAlgorithm::evaluateCoefficients(const double currentScaledValue,
                                                              const ChebyshevFitArc& arc) {
    /* - determine orbit elements from chebychev polynominals */
    double anomalyAngle{}; /* [r] general anomaly angle variable */
    ClassicalElements elements{};
    const double radiusPeriapsis =
        calculateChebyValue(arc.radiusPeriapsisCoefficients.data(), arc.numberChebCoefficients, currentScaledValue) *
        1e3;  // coefficients are in km but module operates in meters
    elements.inclination =
        calculateChebyValue(arc.inclinationCoefficients.data(), arc.numberChebCoefficients, currentScaledValue);
    elements.eccentricity =
        calculateChebyValue(arc.eccentricityCoefficients.data(), arc.numberChebCoefficients, currentScaledValue);
    elements.argPeriapsis =
        calculateChebyValue(arc.argPeriapsisCoefficients.data(), arc.numberChebCoefficients, currentScaledValue);
    elements.rightAscensionAscendingNode =
        calculateChebyValue(arc.raanCoefficients.data(), arc.numberChebCoefficients, currentScaledValue);
    anomalyAngle =
        calculateChebyValue(arc.trueAnomalyCoefficients.data(), arc.numberChebCoefficients, currentScaledValue);

    /*! - determine the true anomaly angle */
    if (arc.anomalyFlag == 0) {
        elements.trueAnomaly = anomalyAngle;
    } else if (elements.eccentricity < 1.0) {
        /* input is mean elliptic anomaly angle */
        elements.trueAnomaly = OrbitalMotion::meanToTrueAnomaly(anomalyAngle, elements.eccentricity);
    } else {
        /* input is mean hyperbolic anomaly angle */
        elements.trueAnomaly = OrbitalMotion::hyperbolicToTrueAnomaly(
            OrbitalMotion::meanToHyperbolicAnomaly(anomalyAngle, elements.eccentricity), elements.eccentricity);
    }

    /*! - determine semi-major axis */
    if (fabs(elements.eccentricity - 1.0) > 1e-12) {
        /* elliptic or hyperbolic case */
        elements.semiMajorAxis = radiusPeriapsis / (1.0 - elements.eccentricity);
    } else {
        /* parabolic case, the elem2rv() function assumes a parabola has a = 0 */
        elements.semiMajorAxis = 0.0;
    }
    return elements;
}

/*! This method takes the current time and computes the state of the object
    using that time and the stored Chebyshev coefficients.  If the time provided
    is outside the specified range, the position vectors rail high/low appropriately.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
EphemerisMsgPayload OEStateEphemAlgorithm::updateState(const uint64_t callTime) {
    auto ephmerisMessageOutput = EphemerisMsgPayload{};
    /*! - Write the output message time */
    ephmerisMessageOutput.timeTag = callTime * 1e-9;
    /*! If all of the radius of periapsis components are zero, this is the central body and should return all zeros*/
    if (std::all_of(this->fitCoefficients[0].radiusPeriapsisCoefficients.begin(),
                    this->fitCoefficients[0].radiusPeriapsisCoefficients.end(),
                    [](double val) { return std::abs(val) < 1e-10; })) {
        return ephmerisMessageOutput;
    }

    auto currentArc = this->findCurrentArc(callTime, this->spacecraftTime);
    auto currentScaledValue = this->scaleEphemerisTime(currentArc);
    auto orbitalElements = this->evaluateCoefficients(currentScaledValue, currentArc);

    /*! - Determine position and velocity vectors */
    auto carteisianState = OrbitalMotion::elementsToCartesianState(this->gravitationalParameter, orbitalElements);
    eigenVectorToCArray(carteisianState.position, ephmerisMessageOutput.r_BdyZero_N);
    eigenVectorToCArray(carteisianState.velocity, ephmerisMessageOutput.v_BdyZero_N);

    return ephmerisMessageOutput;
}

void OEStateEphemAlgorithm::setCentralBodyGravitationalParameter(const double mu) {
    if (mu < 0) {
        throw std::invalid_argument("GravitationalParameter in OEStateEphemAlgorithm must be positive.");
    }
    this->gravitationalParameter = mu;
};

double OEStateEphemAlgorithm::getCentralBodyGravitationalParameter() const { return this->gravitationalParameter; };

void OEStateEphemAlgorithm::setArcNumberOfCoefficients(const unsigned int arcNumber,
                                                       const unsigned int numberOfCoefficients) {
    this->fitCoefficients[arcNumber].numberChebCoefficients = numberOfCoefficients;
};

unsigned int OEStateEphemAlgorithm::getArcNumberOfCoefficients(const unsigned int arcNumber) const {
    return this->fitCoefficients[arcNumber].numberChebCoefficients;
};

void OEStateEphemAlgorithm::setArcMiddleTime(const unsigned int arcNumber, const double timeMiddle) {
    this->fitCoefficients[arcNumber].ephemerisTimeMiddle = timeMiddle;
};

double OEStateEphemAlgorithm::getArcMiddleTime(const unsigned int arcNumber) const {
    return this->fitCoefficients[arcNumber].ephemerisTimeMiddle;
};

void OEStateEphemAlgorithm::setArcRadiusTime(const unsigned int arcNumber, const double timeRadius) {
    this->fitCoefficients[arcNumber].ephemerisTimeRadius = timeRadius;
};

double OEStateEphemAlgorithm::getArcRadiusTime(const unsigned int arcNumber) const {
    return this->fitCoefficients[arcNumber].ephemerisTimeRadius;
};

void OEStateEphemAlgorithm::setArcAnomalyFlag(const unsigned int arcNumber, const unsigned int anomalyFlag) {
    this->fitCoefficients[arcNumber].anomalyFlag = anomalyFlag;
};

unsigned int OEStateEphemAlgorithm::getArcAnomalyFlag(unsigned int arcNumber) const {
    return this->fitCoefficients[arcNumber].anomalyFlag;
};

void OEStateEphemAlgorithm::setArcRadiusPeriapsisCoefficients(
    const unsigned int arcNumber,
    const std::array<double, MAX_OE_COEFF>& radiusPeriapsisCoefficients) {
    this->fitCoefficients[arcNumber].radiusPeriapsisCoefficients = radiusPeriapsisCoefficients;
};

std::array<double, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcRadiusPeriapsisCoefficients(
    const unsigned int arcNumber) {
    return this->fitCoefficients[arcNumber].radiusPeriapsisCoefficients;
};

void OEStateEphemAlgorithm::setArcEccentricityCoefficients(
    const unsigned int arcNumber,
    const std::array<double, MAX_OE_COEFF>& eccentricityCoefficients) {
    this->fitCoefficients[arcNumber].eccentricityCoefficients = eccentricityCoefficients;
};

std::array<double, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcEccentricityCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients[arcNumber].eccentricityCoefficients;
};

void OEStateEphemAlgorithm::setArcInclinationCoefficients(
    const unsigned int arcNumber,
    const std::array<double, MAX_OE_COEFF>& inclinationCoefficients) {
    this->fitCoefficients[arcNumber].inclinationCoefficients = inclinationCoefficients;
};

std::array<double, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcInclinationCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients[arcNumber].inclinationCoefficients;
};

void OEStateEphemAlgorithm::setArcArgPeriapsisCoefficients(
    const unsigned int arcNumber,
    const std::array<double, MAX_OE_COEFF>& argPeriapsisCoefficients) {
    this->fitCoefficients[arcNumber].argPeriapsisCoefficients = argPeriapsisCoefficients;
};

std::array<double, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcArgPeriapsisCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients[arcNumber].argPeriapsisCoefficients;
};

void OEStateEphemAlgorithm::setArcRaanCoefficients(const unsigned int arcNumber,
                                                   const std::array<double, MAX_OE_COEFF>& raanCoefficients) {
    this->fitCoefficients[arcNumber].raanCoefficients = raanCoefficients;
};

std::array<double, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcRaanCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients[arcNumber].raanCoefficients;
};

void OEStateEphemAlgorithm::setArcTrueAnomalyCoefficients(
    const unsigned int arcNumber,
    const std::array<double, MAX_OE_COEFF>& trueAnomalyCoefficients) {
    this->fitCoefficients[arcNumber].trueAnomalyCoefficients = trueAnomalyCoefficients;
};

std::array<double, MAX_OE_COEFF> OEStateEphemAlgorithm::getArcTrueAnomalyCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients[arcNumber].trueAnomalyCoefficients;
};
