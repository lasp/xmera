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

#include "fswAlgorithms/transDetermination/oeStateEphem/oeStateEphem.h"

/*!
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void OEStateEphem::reset(uint64_t callTime)
{
    // check if the required message has not been connected
    assert(!this->clockCorrInMsg.isLinked());
}

ChebyshevFitArc OEStateEphem::findCurrentArc(uint64_t callTime, const TDBVehicleClockCorrelationMsgPayload &localTime) {
    /*! - compute time for fitting interval */
    this->currentEphTime = callTime*NANO2SEC + localTime.ephemerisTime - localTime.vehicleClockTime;

    /*! - select the fitting coefficients for the nearest fit interval */
    signed int nearestArc = 0;
    double smallestTimeDifference = fabs(this->currentEphTime - this->fitCoefficients[0].ephemerisTimeMiddle);
    double timeDifference{};                  /* [s] time difference with respect to an interval mid-point */
    for(int i=1; i<MAX_OE_RECORDS; i++)
    {
        timeDifference = fabs(this->currentEphTime - this->fitCoefficients[i].ephemerisTimeMiddle);
        if(timeDifference < smallestTimeDifference)
        {
            nearestArc = i;
            smallestTimeDifference = timeDifference;
        }
    }

    /*! - determine the scaled fitting time */
    return this->fitCoefficients[nearestArc];
}

double OEStateEphem::scaleEphemerisTime(const ChebyshevFitArc & arc) const {
    double currentScaledValue = (this->currentEphTime - arc.ephemerisTimeMiddle)/arc.ephemerisTimeRadius;
    if(fabs(currentScaledValue) > 1.0)
    {
        currentScaledValue = currentScaledValue/fabs(currentScaledValue);
    }
    return currentScaledValue;
}

ClassicElements OEStateEphem::evaluateCoefficients(const double currentScaledValue, const ChebyshevFitArc &arc) {
/* - determine orbit elements from chebychev polynominals */
    double anomalyAngle{};                    /* [r] general anomaly angle variable */
    ClassicElements elements{};
    elements.rPeriap = calculateChebyValue(arc.radiusPeriapsisCoefficients.data(), arc.numberChebCoefficients,
                                  currentScaledValue);
    elements.i = calculateChebyValue(arc.inclinationCoefficients.data(), arc.numberChebCoefficients,
                                  currentScaledValue);
    elements.e = calculateChebyValue(arc.eccentricityCoefficients.data(), arc.numberChebCoefficients,
                                  currentScaledValue);
    elements.omega = calculateChebyValue(arc.argPeriapsisCoefficients.data(), arc.numberChebCoefficients,
                                  currentScaledValue);
    elements.Omega = calculateChebyValue(arc.raanCoefficients.data(), arc.numberChebCoefficients,
                                  currentScaledValue);
    anomalyAngle = calculateChebyValue(arc.trueAnomalyCoefficients.data(), arc.numberChebCoefficients,
                                   currentScaledValue);

    /*! - determine the true anomaly angle */
    if (arc.anomalyFlag == 0) {
        elements.f = anomalyAngle;
    } else if (elements.e < 1.0) {
        /* input is mean elliptic anomaly angle */
        elements.f = E2f(M2E(anomalyAngle, elements.e), elements.e);
    } else {
        /* input is mean hyperbolic anomaly angle */
        elements.f = H2f(N2H(anomalyAngle, elements.e), elements.e);
    }

    /*! - determine semi-major axis */
    if (fabs(elements.e - 1.0) > 1e-12) {
        /* elliptic or hyperbolic case */
        elements.a = elements.rPeriap/(1.0-elements.e);
    } else {
        /* parabolic case, the elem2rv() function assumes a parabola has a = 0 */
        elements.a = 0.0;
    }
    return elements;
}

/*! This method takes the current time and computes the state of the object
    using that time and the stored Chebyshev coefficients.  If the time provided
    is outside the specified range, the position vectors rail high/low appropriately.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void OEStateEphem::updateState(const uint64_t callTime)
{
    TDBVehicleClockCorrelationMsgPayload localTime = this->clockCorrInMsg();
    auto tmpOutputState = EphemerisMsgPayload();

    auto currentArc  = this->findCurrentArc(callTime, localTime);
    auto currentScaledValue = this->scaleEphemerisTime(currentArc);
    auto orbitalElements = this->evaluateCoefficients(currentScaledValue, currentArc);

    /*! - Determine position and velocity vectors */
    elem2rv(this->gravitationalParameter, &orbitalElements, tmpOutputState.r_BdyZero_N, tmpOutputState.v_BdyZero_N);

    /*! - Write the output message */
    tmpOutputState.timeTag = callTime*NANO2SEC;
    this->stateFitOutMsg.write(&tmpOutputState, moduleID, callTime);
}

void OEStateEphem::setCentralBodyGravitationalParameter(const double mu) {
    this->gravitationalParameter = mu;
};

double OEStateEphem::getCentralBodyGravitationalParameter() const {
    return this->gravitationalParameter;
};

void OEStateEphem::setArcNumberOfCoefficients(const signed int arcNumber, const signed int numberOfCoefficients) {
    this->fitCoefficients[arcNumber].numberChebCoefficients = numberOfCoefficients;
};


signed int OEStateEphem::getArcNumberOfCoefficients(const signed int arcNumber) const {
    return this->fitCoefficients[arcNumber].numberChebCoefficients;
};

void OEStateEphem::setArcMiddleTime(const signed int arcNumber, const double timeMiddle) {
    this->fitCoefficients[arcNumber].ephemerisTimeMiddle = timeMiddle;
};

double OEStateEphem::getArcMiddleTime(const signed int arcNumber) const {
    return this->fitCoefficients[arcNumber].ephemerisTimeMiddle;
};

void OEStateEphem::setArcRadiusTime(const signed int arcNumber, const double timeRadius) {
    this->fitCoefficients[arcNumber].ephemerisTimeRadius = timeRadius;
};

double OEStateEphem::getArcRadiusTime(const signed int arcNumber) const {
    return this->fitCoefficients[arcNumber].ephemerisTimeRadius;
};

void OEStateEphem::setArcAnomalyFlag(const signed int arcNumber, const signed int anomalyFlag) {
    this->fitCoefficients[arcNumber].anomalyFlag = anomalyFlag;
};

signed int OEStateEphem::getArcAnomalyFlag(signed int arcNumber) const {
    return this->fitCoefficients[arcNumber].anomalyFlag;
};

void OEStateEphem::setArcRadiusPeriapsisCoefficients(const signed int arcNumber, const std::array<double, MAX_OE_COEFF> &radiusPeriapsisCoefficients) {
    this->fitCoefficients[arcNumber].radiusPeriapsisCoefficients = radiusPeriapsisCoefficients;
};

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcRadiusPeriapsisCoefficients(const signed int arcNumber) {
    return this->fitCoefficients[arcNumber].radiusPeriapsisCoefficients;
};

void OEStateEphem::setArcEccentricityCoefficients(const signed int arcNumber, const std::array<double, MAX_OE_COEFF> &eccentricityCoefficients) {
    this->fitCoefficients[arcNumber].eccentricityCoefficients = eccentricityCoefficients;
};

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcEccentricityCoefficients(const signed int arcNumber) {
    return this->fitCoefficients[arcNumber].eccentricityCoefficients;
};

void OEStateEphem::setArcInclinationCoefficients(const signed int arcNumber, const std::array<double, MAX_OE_COEFF> &inclinationCoefficients) {
    this->fitCoefficients[arcNumber].inclinationCoefficients = inclinationCoefficients;
};

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcInclinationCoefficients(const signed int arcNumber) {
    return this->fitCoefficients[arcNumber].inclinationCoefficients;
};

void OEStateEphem::setArcArgPeriapsisCoefficients(const signed int arcNumber, const std::array<double, MAX_OE_COEFF> &argPeriapsisCoefficients) {
    this->fitCoefficients[arcNumber].argPeriapsisCoefficients = argPeriapsisCoefficients;
};

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcArgPeriapsisCoefficients(const signed int arcNumber) {
    return this->fitCoefficients[arcNumber].argPeriapsisCoefficients;
};

void OEStateEphem::setArcRaanCoefficients(const signed int arcNumber, const std::array<double, MAX_OE_COEFF> &raanCoefficients) {
    this->fitCoefficients[arcNumber].raanCoefficients = raanCoefficients;
};

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcRaanCoefficients(const signed int arcNumber) {
    return this->fitCoefficients[arcNumber].raanCoefficients;
};

void OEStateEphem::setArcTrueAnomalyCoefficients(const signed int arcNumber, const std::array<double, MAX_OE_COEFF> &trueAnomalyCoefficients) {
    this->fitCoefficients[arcNumber].trueAnomalyCoefficients = trueAnomalyCoefficients;
};

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcTrueAnomalyCoefficients(const signed int arcNumber) {
    return this->fitCoefficients[arcNumber].trueAnomalyCoefficients;
};