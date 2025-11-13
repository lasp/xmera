#ifndef _CLASSIC_ELEMENTS_H
#define _CLASSIC_ELEMENTS_H

/*! @brief Structure used to define classic orbit elements */
typedef struct {
    double a;        //!< object semi-major axis
    double e;        //!< Eccentricity of the orbit
    double i;        //!< inclination of the orbital plane
    double Omega;    //!< Right ascension of the ascending node
    double omega;    //!< Argument of periapsis of the orbit
    double f;        //!< True anomaly of the orbit
    double rmag;     //!< Magnitude of the position vector (extra)
    double alpha;    //!< Inverted semi-major axis (extra)
    double rPeriap;  //!< Radius of periapsis (extra)
    double rApoap;   //!< Radius if apoapsis (extra)
} ClassicElementsMsgPayload;

#endif
