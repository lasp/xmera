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

#ifndef _COB_CONVERT_H_
#define _COB_CONVERT_H_

#include "architecture/messaging/messaging.h"

#include "architecture/msgPayloadDef/CameraModelMsgPayload.h"
#include "architecture/msgPayloadDef/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDef/FilterMsgPayload.h"
#include "architecture/msgPayloadDef/NavAttMsgPayload.h"
#include "architecture/msgPayloadDef/OpNavCOBMsgPayload.h"
#include "architecture/msgPayloadDef/OpNavCOMMsgPayload.h"
#include "architecture/msgPayloadDef/OpNavUnitVecMsgPayload.h"

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "fswAlgorithms/opticalNavigation/cobConverter/cobConverterAlgorithm.h"

/**
 * @enum PhaseAngleCorrectionMethod
 * @brief Phase-angle correction models for converting COB to COM.
 */
enum class PhaseAngleCorrectionMethod { NoCorrection, Lambertian, Binary };

const std::map<PhaseAngleCorrectionMethod, PhaseAngleCorrectionMethodAlgorithm> enumMap = {
    {PhaseAngleCorrectionMethod::NoCorrection, PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg},
    {PhaseAngleCorrectionMethod::Lambertian, PhaseAngleCorrectionMethodAlgorithm::LambertianAlg},
    {PhaseAngleCorrectionMethod::Binary, PhaseAngleCorrectionMethodAlgorithm::BinaryAlg}};

/**
 * @class CobConverter
 * @brief Converts center-of-brightness (COB) pixel measurements into unit vectors
 *        (camera, body, inertial frames), with optional phase-angle correction
 *        and outlier detection.
 */
class CobConverter : public SysModel {
   public:
    CobConverter(PhaseAngleCorrectionMethod method, double radiusObject);
    ~CobConverter() final;

    void updateState(uint64_t currentSimNanos) override;
    void reset(uint64_t currentSimNanos) override;

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

   public:
    // Output messages
    Message<OpNavUnitVecMsgPayload> opnavUnitVecCOBOutMsg;
    Message<OpNavUnitVecMsgPayload> opnavUnitVecCOMOutMsg;
    Message<OpNavCOMMsgPayload> opnavCOMOutMsg;

    // Input messages
    ReadFunctor<OpNavCOBMsgPayload> opnavCOBInMsg;
    ReadFunctor<FilterMsgPayload> opnavFilterInMsg;
    ReadFunctor<CameraModelMsgPayload> cameraConfigInMsg;
    ReadFunctor<NavAttMsgPayload> navAttInMsg;
    ReadFunctor<EphemerisMsgPayload> ephemInMsg;
    ReadFunctor<NavAttMsgPayload> sunInMsg;

   private:
    CobConverterAlgorithm algorithm;
};

#endif
