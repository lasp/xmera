#ifndef AtmoPropsSimMsg_H
#define AtmoPropsSimMsg_H

/*! atmospheric property message definition */
typedef struct {
    double neutralDensity;  //!< kg/m^3 Local neutral particle density
    double localTemp;       //!< K Local avg particle temperature
} AtmoPropsMsgPayload;
#endif
