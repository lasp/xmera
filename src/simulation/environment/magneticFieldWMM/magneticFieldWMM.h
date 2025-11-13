#ifndef WMM_MAGNETIC_FIELD_H
#define WMM_MAGNETIC_FIELD_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <simulation/environment/_GeneralModuleFiles/magneticFieldBase.h>
#include <architecture/utilities/astroConstants.h>
#include "GeomagnetismHeader.h"
#include <architecture/utilities/bskLogging.h>
#include <ctime>

/*! @brief magnetic field WMM class */
class MagneticFieldWMM : public MagneticFieldBase {
   public:
    MagneticFieldWMM();
    ~MagneticFieldWMM();

   private:
    void evaluateMagneticFieldModel(MagneticFieldMsgPayload* msg, double currentTime);
    void initializeWmm();
    void cleanupEarthMagFieldModel();
    void computeWmmField(double decimalYear, double phi, double lambda, double h, double B_M[3]);
    void customreset(uint64_t CurrentClock);
    void customSetEpochFromVariable();
    void decimalYear2Gregorian(double fractionalYear, struct tm* gregorian);
    double gregorian2DecimalYear(double currentTime);

   public:
    std::string dataPath;            //!< -- String with the path to the WMM coefficient file
    double epochDateFractionalYear;  //!< Specified epoch date as a fractional year
    BSKLogger bskLogger;             //!< -- BSK Logging

   private:
    MAGtype_MagneticModel* magneticModels[1];
    MAGtype_MagneticModel* timedMagneticModel;
    MAGtype_Ellipsoid ellip;
    MAGtype_Geoid geoid;
    MAGtype_Date userDate;
};

#endif /* WMM_MAGNETIC_FIELD_H */
