#ifndef POINT_MASS_GRAVITY_MODEL_H
#define POINT_MASS_GRAVITY_MODEL_H

#include <simulation/dynamics/_GeneralModuleFiles/gravityModel.h>

/**
 * The point mass gravity model
 */
class PointMassGravityModel : public GravityModel {
   public:
    /** Does nothing, as the point-mass gravity model has no parameters other than
     * `muBody`, which must be set separately */
    std::optional<std::string> initializeParameters() override { return {}; };

    /** Reads the only necessary parameter (`muBody`) from the given `GravBodyData`*/
    std::optional<std::string> initializeParameters(const GravBodyData&) override;

    /** Returns the gravity acceleration at a position around this body.
     *
     * The position is given in the body-fixed reference frame.
     * Likewise, the resulting acceleration should be given in the
     * body-fixed reference frame.
     */
    Eigen::Vector3d computeField(const Eigen::Vector3d& position_planetFixed) const override;

    /** Returns the gravitational potential energy at a position around this body.
     *
     * The position is given relative to the body and in the inertial
     * reference frame.
     */
    double computePotentialEnergy(const Eigen::Vector3d& positionWrtPlanet_N) const override;

   public:
    double muBody = 0; /**< [m^3/s^2] Gravitation parameter for the planet */
};

#endif /* POINT_MASS_GRAVITY_MODEL_H */
