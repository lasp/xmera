#ifndef FUEL_SLOSH_H
#define FUEL_SLOSH_H



//Fuel tank models
/*! @brief This class is a class that has the defines a generic fuel slosh paricle*/
class FuelSlosh {
public:
    FuelSlosh(){return;};                  //!< -- Contructor
    virtual ~FuelSlosh(){return;};         //!< -- Destructor
    virtual void retrieveMassValue(double integTime){return;}; //!< -- retrieve current mass value of fuelSlosh particle

public:
    double fuelMass = 0.0;                 //!< [kg] mass of fuelSlosh particle
    double massToTotalTankMassRatio = 0.0; //!< -- ratio of fuelSlosh particle mass to total mass of fuel tank
    double fuelMassDot = 0.0;              //!< [kg/s] mass depletion rate of fuelSlosh particle
};


#endif /* FUEL_SLOSH_H */
