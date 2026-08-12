// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FACET_DRAG_DYNAMIC_EFFECTOR_H
#define FACET_DRAG_DYNAMIC_EFFECTOR_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AtmoPropsMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/rigidBodyKinematics.h>

#include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
#include <simulation/dynamics/_GeneralModuleFiles/stateData.h>

#include <Eigen/Dense>
#include <vector>

/*! @brief spacecraft geometry data */
typedef struct {
    std::vector<double> facetAreas;                 //!< vector of facet areas
    std::vector<double> facetCoeffs;                //!< vector of facet coefficients
    std::vector<Eigen::Vector3d> facetNormals_B;    //!< vector of facet normals
    std::vector<Eigen::Vector3d> facetLocations_B;  //!< vector of facet locations
} SpacecraftGeometryData;

/*! @brief faceted atmospheric drag dynamic effector */
class FacetDragDynamicEffector
    : public SysModel
    , public DynamicEffector {
public:
    FacetDragDynamicEffector();
    void linkInStates(DynParamManager &states) override;
    void computeForceTorque(double integTime, double timeStep) override;
    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void addFacet(double area, double dragCoeff, Eigen::Vector3d B_normal_hat, Eigen::Vector3d B_location);

    uint64_t numFacets = 0;                          //!< number of facets
    ReadFunctor<AtmoPropsMsgPayload> atmoDensInMsg;  //!< atmospheric density input message
    StateData* hubSigma;                             //!< -- Hub/Inertial attitude represented by MRP
    StateData* hubVelocity;                          //!< m/s Hub inertial velocity vector
    Eigen::Vector3d v_B;                             //!< m/s local variable to hold the inertial velocity
    Eigen::Vector3d v_hat_B;                         //!< class variable

private:
    bool readInputs();
    void plateDrag();
    void updateDragDir();

    AtmoPropsMsgPayload atmoInData;
    SpacecraftGeometryData scGeometry;  //!< -- Struct to hold spacecraft facet data
};

#endif
