/*
 ISC License

 Copyright (c) 2025, University of Colorado at Boulder

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

#ifndef _COB_CONVERT_ALGORITHM_H_
#define _COB_CONVERT_ALGORITHM_H_

#include "architecture/utilities/avsEigenSupport.h"
#include <Eigen/Dense>
#include <cstdint>

#include "architecture/msgPayloadDef/CameraModelMsgPayload.h"
#include "architecture/msgPayloadDef/FilterMsgPayload.h"
#include "architecture/msgPayloadDef/NavAttMsgPayload.h"
#include "architecture/msgPayloadDef/OpNavCOBMsgPayload.h"
#include "architecture/msgPayloadDef/OpNavCOMMsgPayload.h"
#include "architecture/msgPayloadDef/OpNavUnitVecMsgPayload.h"

/**
 * @enum PhaseAngleCorrectionMethodAlgorithm
 * @brief Phase-angle correction models for converting COB to COM.
 */
enum class PhaseAngleCorrectionMethodAlgorithm { NoCorrectionAlg, LambertianAlg, BinaryAlg };

/**
 * @class CobConverterAlgorithm
 * @brief Converts center-of-brightness (COB) pixel measurements into unit vectors
 *        (camera, body, inertial frames), with optional phase-angle correction
 *        and outlier detection.
 */
class CobConverterAlgorithm {
   public:
    CobConverterAlgorithm(PhaseAngleCorrectionMethodAlgorithm method, double radiusObject);
    ~CobConverterAlgorithm();

    std::tuple<OpNavUnitVecMsgPayload, OpNavCOMMsgPayload> updateState(const uint64_t currentSimNanos,
                                                                       const CameraModelMsgPayload &cameraSpecs,
                                                                       const OpNavCOBMsgPayload &cobMsgBuffer,
                                                                       const NavAttMsgPayload &navAttBuffer,
                                                                       const NavAttMsgPayload &sunBuffer,
                                                                       const FilterMsgPayload &filterMsgBuffer);

    void setRadius(double radius);
    double getRadius() const;
    void setRadiusUncertainty(double radiusUncertainty);
    double getRadiusUncertainty() const;
    void setAttitudeCovariance(const Eigen::Matrix3d &covAtt_BN_B);
    Eigen::Matrix3d getAttitudeCovariance() const;
    void setNumStandardDeviations(double num);
    double getNumStandardDeviations() const;
    void setStandardDeviation(double num);
    double getStandardDeviation() const;
    bool isStandardDeviationSpecified() const;
    void enableOutlierDetection();
    void disableOutlierDetection();
    bool isOutlierDetectionEnabled() const;

   private:
    void cobOutlierDetection(const FilterMsgPayload &filterMsgBuffer);
    void computeCameraParameters(const CameraModelMsgPayload &cameraSpecs);
    void computeRotations(const NavAttMsgPayload &navAttBuffer);
    void computePhaseAngleCorrection(const FilterMsgPayload &filterBuffer, const NavAttMsgPayload &sunBuffer);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> computeCentersOfInterest(const OpNavCOBMsgPayload &cobMsgBuffer) const;
    void computeRelevantVectors(const Eigen::Vector3d &centerOfBrightness, const Eigen::Vector3d &centerOfMass);
    void computeCameraFrameUncertainty(const FilterMsgPayload &filterMsgBuffer, double pixelsFound);
    std::tuple<OpNavUnitVecMsgPayload, OpNavCOMMsgPayload> populateOutputMessages(
        uint64_t timeTag,
        const Eigen::Vector3d &centerOfMass,
        const Eigen::Vector3d &centerOfBrightness,
        OpNavUnitVecMsgPayload &uVecCOBMsgBuffer,
        OpNavCOMMsgPayload &comMsgBuffer);

    PhaseAngleCorrectionMethodAlgorithm phaseAngleCorrectionMethod;
    double objectRadius{};
    double objectRadiusUncertainty{};
    Eigen::Matrix3d covarAtt_BN_B{};
    Eigen::Matrix3d dcm_NC{};
    Eigen::Matrix3d dcm_CB{};
    Eigen::Matrix3d dcm_BN{};
    Eigen::Matrix3d cameraCalibrationMatrix{};
    Eigen::Matrix3d cameraCalibrationMatrixInverse{};
    Eigen::Matrix3d covar_B{};
    double numStandardDeviations = 3;
    double standardDeviation{};
    bool specifiedStandardDeviation{};
    bool performOutlierDetection{};
    bool validCOM = false;
    double dX{};
    double X{};
    double Y{};
    double ifov_x{};
    double ifov_y{};
    double Rc = 0;
    double gamma = 0;
    double phi = 0;
    double alphaPA = 0;
    Eigen::Vector3d rhatCOB_C{};
    Eigen::Vector3d rhatCOM_C{};
    Eigen::Vector3d sc_position{};
    Eigen::Vector3d shat_N{};
    double rhatCOBNorm = 0;
    ;
    double spacecraftRange = 0;
    int cameraId = 0;
    bool goodOutlierCheck = true;
};

#endif
