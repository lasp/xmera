// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FACET_SRP_DYNAMIC_EFFECTOR_H
#define FACET_SRP_DYNAMIC_EFFECTOR_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
#include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenMRP.h>
#include <architecture/utilities/eigenSupport.h>

#include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
#include <simulation/dynamics/_GeneralModuleFiles/stateData.h>

#include <Eigen/Dense>
#include <vector>

/*! @brief Spacecraft Geometry Data */
typedef struct {
    std::vector<double> facetAreas;               //!< [m^2] Vector of facet areas
    std::vector<double> facetSpecCoeffs;          //!< Vector of facet spectral reflection optical coefficients
    std::vector<double> facetDiffCoeffs;          //!< Vector of facet diffuse reflection optical coefficients
    std::vector<Eigen::Vector3d> facetNormals_B;  //!< Vector of facet normals expressed in B frame components
    std::vector<Eigen::Vector3d>
        facetLocationsPntB_B;  //!< [m] Vector of facet COP locations wrt point B expressed in B frame components
    std::vector<Eigen::Vector3d> facetRotAxes_B;  //!< [m] Vector of facet rotation axes expressed in B frame components
} FacetedSRPSpacecraftGeometryData;

/*! @brief Faceted Solar Radiation Pressure Dynamic Effector */
class FacetSRPDynamicEffector
    : public SysModel
    , public DynamicEffector {
public:
    FacetSRPDynamicEffector();                            //!< The module constructor
    ~FacetSRPDynamicEffector();                           //!< The module destructor
    void linkInStates(DynParamManager &states) override;  //!< Method for giving the effector access to the hub states
    void computeForceTorque(
        double callTime,
        double timeStep
    ) override;                                     //!< Method for computing the SRP force and torque about point B
    void reset(uint64_t currentSimNanos) override;  //!< Reset method
    void addFacet(
        double area,
        double specCoeff,
        double diffCoeff,
        Eigen::Vector3d normal_B,
        Eigen::Vector3d locationPntB_B,
        Eigen::Vector3d rotAxis_B
    );  //!< Method for adding facets to the spacecraft geometry structure
    void addArticulatedFacet(Message<HingedRigidBodyMsgPayload>* tmpMsg);
    void ReadMessages();

    size_t numFacets;                                  //!< Total number of spacecraft facets
    size_t numArticulatedFacets;                       //!< Number of articulated facets
    ReadFunctor<SpicePlanetStateMsgPayload> sunInMsg;  //!< Sun spice ephemeris input message

    BSKLogger bskLogger;  //!< -- BSK Logging

private:
    std::vector<ReadFunctor<HingedRigidBodyMsgPayload>>
        articulatedFacetDataInMsgs;                  //!< Articulated facet angle data input message
    std::vector<double> facetArticulationAngleList;  //!< [rad] Vector of facet rotation angles
    FacetedSRPSpacecraftGeometryData scGeometry;     //!< Spacecraft facet data structure
    Eigen::Vector3d r_SN_N;                          //!< [m] Sun inertial position vector
    StateData* hubPosition;                          //!< [m] Hub inertial position vector
    StateData* hubSigma;                             //!< Hub MRP inertial attitude
    bool facetAngleMsgRead;  //!< Boolean variable signaling that the facet articulation messages are read
};

#endif
