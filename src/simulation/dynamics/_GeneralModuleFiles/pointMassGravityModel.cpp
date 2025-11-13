#include "pointMassGravityModel.h"
#include "gravityEffector.h"

std::optional<std::string> PointMassGravityModel::initializeParameters(const GravBodyData& body) {
    this->muBody = body.mu;
    return this->initializeParameters();
}

Eigen::Vector3d PointMassGravityModel::computeField(const Eigen::Vector3d& position_planetFixed) const {
    const double rMag = position_planetFixed.norm();
    return -position_planetFixed * this->muBody / (rMag * rMag * rMag);
}

double PointMassGravityModel::computePotentialEnergy(const Eigen::Vector3d& positionWrtPlanet_N) const {
    return -this->muBody / positionWrtPlanet_N.norm();
}
