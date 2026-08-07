// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "cielimInterface.h"

/*! Reset module time-tags, message freshness, and zmq connector
 @param currentSimNanos The current sim time in nanoseconds
*/
void CielimInterface::reset(uint64_t currentSimNanos) {
    if (this->opNavMode != ClosedLoopMode::OPEN_LOOP || this->liveStream) {
        this->imagePointer = nullptr;
        if (!this->connector.isConnected()) { this->connector.connect(); }
    }

    /*! Check spacecraft input message */
    if (this->spacecraftMessage.isLinked()) {
        this->spacecraftMessageStatus.dataFresh = false;
        this->spacecraftMessageStatus.lastTimeTag = 0xFFFF'FFFF'FFFF'FFFF;
    }

    /*! Check Camera input messages */
    if (this->cameraModelMessage.isLinked()) {
        this->cameraModelMessageStatus.dataFresh = false;
        this->cameraModelMessageStatus.lastTimeTag = 0xFFFF'FFFF'FFFF'FFFF;
    }

    /*! Check diagnostic input messages */
    if (this->imageDiagnosticsMessage.isLinked()) {
        this->imageDiagnosticsMessageStatus.dataFresh = false;
        this->imageDiagnosticsMessageStatus.lastTimeTag = 0xFFFF'FFFF'FFFF'FFFF;
    }

    /*! Check Camera rendering information messages */
    if (this->cameraRenderingMessage.isLinked()) {
        this->cameraRenderingMessageStatus.dataFresh = false;
        this->cameraRenderingMessageStatus.lastTimeTag = 0xFFFF'FFFF'FFFF'FFFF;
    }

    /*! Check asteroid parameter information messages */
    if (this->epochMessage.isLinked()) {
        this->epochMessageStatus.dataFresh = false;
        this->epochMessageStatus.lastTimeTag = 0xFFFF'FFFF'FFFF'FFFF;
    }

    this->epochMessageStatus.dataFresh = false;

    /*! Check asteroid parameter information messages */
    MessageStatus celestialParametersStatus;
    celestialParametersStatus.dataFresh = false;
    celestialParametersStatus.lastTimeTag = 0xFFFF'FFFF'FFFF'FFFF;
    this->celestialParametersMessageStatus.clear();
    for (int c = 0; c < this->celestialBodiesList.size(); ++c) {
        this->celestialParametersMessageStatus.push_back(celestialParametersStatus);
    }

    /*! Check Spice input message */
    MessageStatus spiceStatus;
    spiceStatus.dataFresh = true;
    spiceStatus.lastTimeTag = 0xFFFF'FFFF'FFFF'FFFF;
    this->spiceBodyMessageStatus.clear();
    for (size_t c = 0; c < this->celestialBodiesList.size(); ++c) {
        /*! set default zero translation and rotation states */
        SpicePlanetStateMsgPayload logMsg = {};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) { logMsg.J20002Pfix[i][j] = (i == j) ? 1.0 : 0.0; }
        }
        strcpy(logMsg.PlanetName, this->celestialBodiesList.at(c).name.c_str());
        this->spiceBodyMessageStatus.push_back(spiceStatus);
        this->spiceBodyPayloads.push_back(logMsg);
    }

    this->frameNumber = -1;
    if (this->saveFile) {
        std::string temporaryFilename = this->protoFilename;
        for (int i = 1; i < 1'000; ++i) {
            if (!std::filesystem::exists(temporaryFilename)) {
                this->protoFilename = temporaryFilename;
                break;
            }
            temporaryFilename = this->protoFilename + "_" + std::to_string(i);
        }
        this->outputStream = std::ofstream(this->protoFilename, std::ios_base::app | std::ios_base::binary);
    }
}

/*! Read all the relevant Basilisk messages in order to store the data locally before packaging in a protobuffer
 @return void
 * */
void CielimInterface::readBskMessages() {
    /*! Read spacecraft state msg */
    if (this->spacecraftMessage.isLinked()) {
        SCStatesMsgPayload localSCStateArray = this->spacecraftMessage();
        if (this->spacecraftMessage.isWritten()
            && this->spacecraftMessage.timeWritten() != this->spacecraftMessageStatus.lastTimeTag) {
            this->spacecraftMessageStatus.lastTimeTag = this->spacecraftMessage.timeWritten();
            this->spacecraftMessageStatus.dataFresh = true;
        }
        this->spacecraftPayload = localSCStateArray;
    }

    /*! Read camera model message */
    if (this->cameraModelMessage.isLinked()) {
        CameraModelMsgPayload localCameraConfigArray = this->cameraModelMessage();
        if (this->cameraModelMessage.isWritten()
            && this->cameraModelMessage.timeWritten() != this->cameraModelMessageStatus.lastTimeTag) {
            this->cameraModelMessageStatus.lastTimeTag = this->cameraModelMessage.timeWritten();
            this->cameraModelMessageStatus.dataFresh = true;
        }
        this->cameraModelPayload = localCameraConfigArray;

        /*! Read image diagnostics message */
        if (this->imageDiagnosticsMessage.isLinked()) {
            if (this->imageDiagnosticsMessage.isWritten()
                && this->imageDiagnosticsMessage.timeWritten() != this->imageDiagnosticsMessageStatus.lastTimeTag) {
                this->imageDiagnosticsMessageStatus.lastTimeTag = this->imageDiagnosticsMessage.timeWritten();
                this->imageDiagnosticsMessageStatus.dataFresh = true;
            }
            this->imageDiagnosticsPayload = this->imageDiagnosticsMessage();
        }
    }

    /*! Read camera rendering message */
    if (this->cameraRenderingMessage.isLinked()) {
        CameraRenderingMsgPayload cameraRenderingArray = this->cameraRenderingMessage();
        if (this->cameraRenderingMessage.isWritten()
            && this->cameraRenderingMessage.timeWritten() != this->cameraRenderingMessageStatus.lastTimeTag) {
            this->cameraRenderingMessageStatus.lastTimeTag = this->cameraRenderingMessage.timeWritten();
            this->cameraRenderingMessageStatus.dataFresh = true;
        }
        this->cameraRenderingPayload = cameraRenderingArray;
    }

    /*! Read celestial body parameter message */
    for (size_t i = 0; i < this->celestialBodiesList.size(); ++i) {
        if (this->celestialBodiesList.at(i).celestialParametersMessage.isLinked()) {
            CelestialBodyParametersMsgPayload celestialParamArray =
                this->celestialBodiesList.at(i).celestialParametersMessage();
            if (this->celestialBodiesList.at(i).celestialParametersMessage.isWritten()
                && this->celestialBodiesList.at(i).celestialParametersMessage.timeWritten()
                       != this->celestialParametersMessageStatus[i].lastTimeTag) {
                this->celestialParametersMessageStatus[i].lastTimeTag =
                    this->celestialBodiesList.at(i).celestialParametersMessage.timeWritten();
                this->celestialParametersMessageStatus[i].dataFresh = true;
            }
            this->celestialBodiesList.at(i).celestialParametersPayload = celestialParamArray;
        }
    }

    /*! Read sim epoch msg */
    if (this->epochMessage.isLinked()) {
        EpochMsgPayload epochMessageBuffer = this->epochMessage();
        if (this->epochMessage.isWritten()
            && this->epochMessage.timeWritten() != this->epochMessageStatus.lastTimeTag) {
            this->epochMessageStatus.lastTimeTag = this->epochMessage.timeWritten();
            this->epochMessageStatus.dataFresh = true;
        }
        this->epochPayload = epochMessageBuffer;
    }

    /*! Read celestial bodies messages */
    for (size_t i = 0; i < this->celestialBodiesList.size(); ++i) {
        if (this->celestialBodiesList.at(i).spiceStateMessage.isLinked()) {
            // If the spice msg is not linked then the default zero planet ephemeris is used
            SpicePlanetStateMsgPayload localSpiceArray = this->celestialBodiesList.at(i).spiceStateMessage();
            if (this->celestialBodiesList.at(i).spiceStateMessage.isWritten()
                && this->celestialBodiesList.at(i).spiceStateMessage.timeWritten()
                       != this->spiceBodyMessageStatus[i].lastTimeTag) {
                this->spiceBodyMessageStatus[i].lastTimeTag =
                    this->celestialBodiesList.at(i).spiceStateMessage.timeWritten();
                this->spiceBodyMessageStatus[i].dataFresh = true;
                this->celestialBodiesList.at(i).spiceStatePayload = localSpiceArray;
            }
        }
    }
}

/*! Write a protobuffer with the information from the simulation .
 @param currentSimNanos The current sim time in nanoseconds
 */
void CielimInterface::writeProtobuffer(uint64_t currentSimNanos) {
    auto visPayload = cielimMessage::CielimMessage();

    /*! Write timestamp output msg */
    auto* time = new cielimMessage::TimeStamp();
    time->set_framenumber(this->frameNumber);
    time->set_simtimeelapsed((double) currentSimNanos);
    visPayload.set_allocated_currenttime(time);

    /*! write epoch msg */
    if (this->epochMessageStatus.dataFresh) {
        auto* epoch = new cielimMessage::EpochDateTime();
        epoch->set_year(this->epochPayload.year);
        epoch->set_month(this->epochPayload.month);
        epoch->set_day(this->epochPayload.day);
        epoch->set_hours(this->epochPayload.hours);
        epoch->set_minutes(this->epochPayload.minutes);
        epoch->set_seconds(this->epochPayload.seconds);
        visPayload.set_allocated_epoch(epoch);
        this->epochMessageStatus.dataFresh = false;
    }

    /*! Write spice output msgs */
    for (size_t k = 0; k < this->spiceBodyMessageStatus.size(); ++k) {
        if (this->spiceBodyMessageStatus[k].dataFresh) {
            cielimMessage::CelestialBody* celestialBody = visPayload.add_celestialbodies();
            celestialBody->set_bodyname(this->celestialBodiesList.at(k).name);
            for (int i = 0; i < 3; i++) {
                celestialBody->add_position(this->celestialBodiesList.at(k).spiceStatePayload.PositionVector[i]);
                celestialBody->add_velocity(this->celestialBodiesList.at(k).spiceStatePayload.VelocityVector[i]);

                celestialBody->add_attitude(this->celestialBodiesList.at(k).spiceStatePayload.J20002Pfix[i][0]);
                celestialBody->add_attitude(this->celestialBodiesList.at(k).spiceStatePayload.J20002Pfix[i][1]);
                celestialBody->add_attitude(this->celestialBodiesList.at(k).spiceStatePayload.J20002Pfix[i][2]);
            }
            celestialBody->set_centralbody(this->celestialBodiesList.at(k).isCentralBody);
            if (this->celestialBodiesList.at(k).celestialParametersMessage.isLinked()
                && this->celestialParametersMessageStatus[k].dataFresh) {
                auto* meshModel = new cielimMessage::MeshModel();
                auto* perlinNoise = new cielimMessage::PerlinNoise();
                auto* reflectanceModel = new cielimMessage::ReflectanceModel();
                perlinNoise->set_octavecount(
                    this->celestialBodiesList.at(k).celestialParametersPayload.perlinNoiseOctaveCount
                );
                perlinNoise->set_baseamplitude(
                    this->celestialBodiesList.at(k).celestialParametersPayload.perlinNoiseBaseAmplitude
                );
                perlinNoise->set_basefrequency(
                    this->celestialBodiesList.at(k).celestialParametersPayload.perlinNoiseBaseFrequency
                );
                perlinNoise->set_persistence(
                    this->celestialBodiesList.at(k).celestialParametersPayload.perlinNoisePersistence
                );
                std::string brdfModelName = this->celestialBodiesList.at(k).celestialParametersPayload.brdf;
                reflectanceModel->set_brdfmodel(brdfModelName);
                reflectanceModel->set_isotropicscattering(
                    this->celestialBodiesList.at(k).celestialParametersPayload.isotropicScattering
                );
                for (int i = 0; i < MAX_PARAMETER_LENGTH; ++i) {
                    reflectanceModel->add_reflectanceparameters(
                        this->celestialBodiesList.at(k).celestialParametersPayload.reflectanceParameters[i]
                    );
                }
                meshModel->set_meanradius(this->celestialBodiesList.at(k).celestialParametersPayload.meanRadius);
                meshModel->set_geometricalbedo(
                    this->celestialBodiesList.at(k).celestialParametersPayload.geometricAlbedo
                );
                meshModel->set_allocated_perlinnoise(perlinNoise);
                meshModel->set_allocated_refmodel(reflectanceModel);
                for (int i = 0; i < 3; ++i) {
                    meshModel->add_principalaxisdistortion(
                        this->celestialBodiesList.at(k).celestialParametersPayload.principalAxisDistortion[i]
                    );
                    meshModel->add_inertialtobodymrp(
                        this->celestialBodiesList.at(k).celestialParametersPayload.sigma_BN[i]
                    );
                }
                meshModel->set_proceduralrocks(
                    this->celestialBodiesList.at(k).celestialParametersPayload.proceduralRocks
                );
                meshModel->set_shapemodel(this->celestialBodiesList.at(k).celestialParametersPayload.shapeModel);
                celestialBody->set_allocated_model(meshModel);
            }
        }
    }

    /*! Write spacecraft state output msg */
    if (this->spacecraftMessage.isLinked() && this->spacecraftMessageStatus.dataFresh) {
        auto* spacecraft = new cielimMessage::Spacecraft();
        spacecraft->set_spacecraftname("cielim_sat");
        for (int i = 0; i < 3; i++) {
            spacecraft->add_position(this->spacecraftPayload.r_BN_N[i]);
            spacecraft->add_velocity(this->spacecraftPayload.v_BN_N[i]);
            spacecraft->add_attitude(this->spacecraftPayload.sigma_BN[i]);
        }
        visPayload.set_allocated_spacecraft(spacecraft);
    }
    /*! Write camera output msg */
    if ((this->cameraModelMessage.isLinked() && this->cameraModelMessageStatus.dataFresh)
        || this->cameraModelPayload.cameraId >= 0) {
        /*! This corrective attitude allows UE to place the camera as is expected by the python setting.
         * UE5 has a -x pointing camera, with z vertical on the sensor, and y horizontal which is not the OpNav frame:
         * z point, x horizontal, y vertical (down) */
        auto* camera = new cielimMessage::CameraModel();
        auto* lensModel = new cielimMessage::LensModel();
        auto* sensorModel = new cielimMessage::SensorModel();
        auto* quantumEfficiency = new cielimMessage::QuantumEfficiency();
        auto* dataFormat = new cielimMessage::ImageFormat();

        camera->set_cameraid(this->cameraModelPayload.cameraId);
        camera->set_parentname(this->cameraModelPayload.parentName);
        for (int j = 0; j < 3; j++) {
            camera->add_bodyframetocameramrp(this->cameraModelPayload.bodyToCameraMrp[j]);
            camera->add_camerapositioninbody(this->cameraModelPayload.cameraBodyFramePosition[j]);
        }

        for (int j = 0; j < 2; j++) {
            sensorModel->add_resolution(this->cameraModelPayload.resolution[j]);
            lensModel->add_fieldofview(this->cameraModelPayload.fieldOfView[j]);
        }

        lensModel->set_focallength(this->cameraModelPayload.focalLength);
        lensModel->set_pointspreadfunction(this->cameraModelPayload.gaussianPointSpreadFunction);
        lensModel->set_apertureradius(this->cameraModelPayload.apertureRadius);
        for (int j = 0; j < MAX_POLY_COEFF; j++) {
            lensModel->add_horizontalvignetting(this->cameraModelPayload.horizontalVignetting[j]);
            lensModel->add_verticalvignetting(this->cameraModelPayload.verticalVignetting[j]);
        }
        // The single lens transmission is replicated across the three wavelength sample points.
        lensModel->set_transmission1(this->cameraModelPayload.transmission);
        lensModel->set_transmission2(this->cameraModelPayload.transmission);
        lensModel->set_transmission3(this->cameraModelPayload.transmission);
        // distortion[] maps positionally onto the radial (k1, k2, k3) and tangential (p1, p2) terms.
        lensModel->set_distortionk1(this->cameraModelPayload.distortion[0]);
        lensModel->set_distortionk2(this->cameraModelPayload.distortion[1]);
        lensModel->set_distortionk3(this->cameraModelPayload.distortion[2]);
        lensModel->set_distortionp1(this->cameraModelPayload.distortion[3]);
        lensModel->set_distortionp2(this->cameraModelPayload.distortion[4]);

        sensorModel->set_renderrate(this->cameraModelPayload.renderRate);
        sensorModel->set_exposuretime(this->cameraModelPayload.exposureTime);
        sensorModel->set_readnoise(this->cameraModelPayload.readNoise);
        sensorModel->set_shotnoise(this->cameraModelPayload.shotNoise);
        sensorModel->set_darkcurrent(this->cameraModelPayload.darkCurrent);
        sensorModel->set_systemgain(this->cameraModelPayload.systemGain);
        sensorModel->set_sensorwidth(this->cameraModelPayload.sensorWidth);
        sensorModel->set_sensorheight(this->cameraModelPayload.sensorHeight);
        sensorModel->set_fullwellcapacity(this->cameraModelPayload.fullWellCapacity);
        sensorModel->set_gamma(this->cameraModelPayload.gammaCorrection);
        sensorModel->set_darkcurrentpattern(this->cameraModelPayload.darkCurrentPattern);
        sensorModel->set_darkcurrentstddeviation(this->cameraModelPayload.darkCurrentStdDeviation);
        sensorModel->set_isgrayscale(this->cameraModelPayload.isGrayscale);
        sensorModel->set_pixeldefectpattern(this->cameraModelPayload.pixelDefectPattern);
        sensorModel->set_stuckpixelrate(this->cameraModelPayload.stuckPixelRate);
        sensorModel->set_deadpixelrate(this->cameraModelPayload.deadPixelRate);

        quantumEfficiency->set_integrationweightfactor(this->cameraModelPayload.integrationWeightFactor);
        quantumEfficiency->set_redvalue1(this->cameraModelPayload.redQuantumEfficiency[0]);
        quantumEfficiency->set_redvalue2(this->cameraModelPayload.redQuantumEfficiency[1]);
        quantumEfficiency->set_redvalue3(this->cameraModelPayload.redQuantumEfficiency[2]);
        quantumEfficiency->set_greenvalue1(this->cameraModelPayload.greenQuantumEfficiency[0]);
        quantumEfficiency->set_greenvalue2(this->cameraModelPayload.greenQuantumEfficiency[1]);
        quantumEfficiency->set_greenvalue3(this->cameraModelPayload.greenQuantumEfficiency[2]);
        quantumEfficiency->set_bluevalue1(this->cameraModelPayload.blueQuantumEfficiency[0]);
        quantumEfficiency->set_bluevalue2(this->cameraModelPayload.blueQuantumEfficiency[1]);
        quantumEfficiency->set_bluevalue3(this->cameraModelPayload.blueQuantumEfficiency[2]);
        sensorModel->set_allocated_qecurve(quantumEfficiency);

        if (std::string{this->cameraModelPayload.imageFormat} == "RAW") {
            switch (this->cameraModelPayload.bitDepth) {
            case 8: dataFormat->set_format(cielimMessage::ImageFormat_Format_RAW_8); break;
            case 12: dataFormat->set_format(cielimMessage::ImageFormat_Format_RAW_12); break;
            case 16: dataFormat->set_format(cielimMessage::ImageFormat_Format_RAW_16); break;
            default: dataFormat->set_format(cielimMessage::ImageFormat_Format_RAW_8); break;
            }
        } else {
            dataFormat->set_format(cielimMessage::ImageFormat_Format_PNG);
        }

        camera->set_allocated_lensmodel(lensModel);
        camera->set_allocated_sensormodel(sensorModel);
        camera->set_allocated_imageformat(dataFormat);

        /*! Write diagnostics msg */
        if (this->imageDiagnosticsMessage.isLinked() && this->imageDiagnosticsMessageStatus.dataFresh) {
            auto* areaOfInterest = new cielimMessage::AreaOfInterest();
            areaOfInterest->set_centerx(this->imageDiagnosticsPayload.areaOfInterestCenter[0]);
            areaOfInterest->set_centery(this->imageDiagnosticsPayload.areaOfInterestCenter[1]);
            areaOfInterest->set_width(this->imageDiagnosticsPayload.areaOfInterestWidthHeight[0]);
            areaOfInterest->set_height(this->imageDiagnosticsPayload.areaOfInterestWidthHeight[1]);
            areaOfInterest->set_threshold(this->imageDiagnosticsPayload.threshold);

            camera->set_allocated_areaofinterest(areaOfInterest);
        }

        visPayload.set_allocated_camera(camera);
    }

    if (this->cameraRenderingMessage.isLinked() && this->cameraRenderingMessageStatus.dataFresh) {
        auto* rendering = new cielimMessage::RenderingModel();
        rendering->set_wavelength1(this->cameraRenderingPayload.wavelengths[0]);
        rendering->set_wavelength2(this->cameraRenderingPayload.wavelengths[1]);
        rendering->set_wavelength3(this->cameraRenderingPayload.wavelengths[2]);
        rendering->set_cosmicraystddeviation(this->cameraRenderingPayload.cosmicRayStdDeviation);

        auto* strayLightModel = new cielimMessage::StrayLightModel();
        strayLightModel->set_enabled(this->cameraRenderingPayload.strayLightEnabled);
        strayLightModel->set_coresize(this->cameraRenderingPayload.strayLightCoreSize);
        strayLightModel->set_ghostsize(this->cameraRenderingPayload.strayLightGhostSize);
        strayLightModel->set_ghosttransmittance(this->cameraRenderingPayload.strayLightGhostTransmittance);
        strayLightModel->set_ghost1relativesize(this->cameraRenderingPayload.strayLightGhost1RelativeSize);
        strayLightModel->set_ghost2relativesize(this->cameraRenderingPayload.strayLightGhost2RelativeSize);
        strayLightModel->set_ghost3relativesize(this->cameraRenderingPayload.strayLightGhost3RelativeSize);
        strayLightModel->set_ghost4relativesize(this->cameraRenderingPayload.strayLightGhost4RelativeSize);
        strayLightModel->set_ghostbrightnesssizeexponent(
            this->cameraRenderingPayload.strayLightGhostBrightnessSizeExponent
        );
        strayLightModel->set_coronafalloffexponent(this->cameraRenderingPayload.strayLightCoronaFalloffExponent);
        strayLightModel->set_coronaintensity(this->cameraRenderingPayload.strayLightCoronaIntensity);
        strayLightModel->set_baffleshieldangle(this->cameraRenderingPayload.strayLightBaffleShieldAngle);
        strayLightModel->set_intensity(this->cameraRenderingPayload.strayLightIntensity);
        strayLightModel->set_numrays(this->cameraRenderingPayload.strayLightNumRays);
        strayLightModel->set_raysharpness(this->cameraRenderingPayload.strayLightRaySharpness);
        strayLightModel->set_rayweight(this->cameraRenderingPayload.strayLightRayWeight);
        rendering->set_allocated_straylightmodel(strayLightModel);

        rendering->set_starfield(this->cameraRenderingPayload.starField);
        rendering->set_rendering(this->cameraRenderingPayload.rendering);
        rendering->set_enablesmear(this->cameraRenderingPayload.smear);
        visPayload.set_allocated_renderparameters(rendering);
    }

    /*! Enter in lock-step with the vizard to simulate a camera */
    /*!--OpNavMode set to ALL_FRAMES is to stay in lock-step with the viz at all time steps. It is a slower run,
     * but provides visual capabilities during OpNav */
    /*!--OpNavMode set to REQUESTED_FRAMES is a faster mode in which the viz only steps forward to the BSK time step
     * if an image is requested. This is a faster run but nothing can be visualized post-run */
    if (this->opNavMode == ClosedLoopMode::ALL_FRAMES
        || (this->opNavMode == ClosedLoopMode::REQUESTED_FRAMES && this->shouldRequestACameraImage(currentSimNanos))
        || this->liveStream) {
        this->connector.send(visPayload);

        /*! - If the camera is requesting periodic images, request them */
        if (this->opNavMode != ClosedLoopMode::OPEN_LOOP && currentSimNanos % this->cameraModelPayload.renderRate == 0
            && this->cameraModelPayload.renderRate > 0) {
            this->requestImage(currentSimNanos);
        }
        if (this->shouldRequestACameraImage(currentSimNanos)) { this->connector.ping(); }
    }

    /*!  Write protobuf to file */
    if (this->saveFile) { google::protobuf::util::SerializeDelimitedToOstream(visPayload, &this->outputStream); }
}

/*! UpdateState
 @param currentSimNanos The current sim time
 */
void CielimInterface::updateState(uint64_t currentSimNanos) {
    this->frameNumber += 1;
    this->readBskMessages();
    if (currentSimNanos > 0) { this->writeProtobuffer(currentSimNanos); }
}

/*! Determine if the module should request and image
 @param currentSimNanos The current sim time
 * */
bool CielimInterface::shouldRequestACameraImage(uint64_t currentSimNanos) const {
    if (currentSimNanos % this->cameraModelPayload.renderRate == 0 && this->cameraModelPayload.renderRate > 0) {
        return true;
    }
    return false;
}

/*! Issue an image request
 @param currentSimNanos The current sim time
 * */
void CielimInterface::requestImage(uint64_t currentSimNanos) {
    auto imageData = this->connector.requestImage(this->cameraModelPayload.cameraId);
    this->imagePointer = &imageData.imageBuffer;

    CameraImageMsgPayload imagePayload = {};
    imagePayload.timeTag = currentSimNanos;
    imagePayload.valid = 0;
    imagePayload.imagePointer = imageData.imageBuffer;
    imagePayload.imageBufferLength = imageData.imageBufferLength;
    imagePayload.cameraID = this->cameraModelPayload.cameraId;
    imagePayload.imageType = 3;
    if (imageData.imageBufferLength > 0) { imagePayload.valid = 1; }
    this->imageOutMessage.write(imagePayload, this->moduleID, currentSimNanos);

    OpNavCOBMsgPayload centerOfBrightnessPayload = {};
    centerOfBrightnessPayload.timeTag = currentSimNanos;
    centerOfBrightnessPayload.cameraID = this->cameraModelPayload.cameraId;
    if (imageData.centerOfBrightness) {
        centerOfBrightnessPayload.valid = true;
        centerOfBrightnessPayload.centerOfBrightness[0] = imageData.centerOfBrightness.value()[0] + 0.5;
        centerOfBrightnessPayload.centerOfBrightness[1] = imageData.centerOfBrightness.value()[1] + 0.5;
    }
    this->centerOfBrightnessOutMessage.write(centerOfBrightnessPayload, this->moduleID, currentSimNanos);

    ImageDiagnosticsPayload imageDiagnosticsOutputPayload = {};
    imageDiagnosticsOutputPayload.threshold = this->imageDiagnosticsPayload.threshold;
    imageDiagnosticsOutputPayload.areaOfInterestCenter[0] = this->imageDiagnosticsPayload.areaOfInterestCenter[0];
    imageDiagnosticsOutputPayload.areaOfInterestCenter[1] = this->imageDiagnosticsPayload.areaOfInterestCenter[1];
    imageDiagnosticsOutputPayload.areaOfInterestWidthHeight[0] =
        this->imageDiagnosticsPayload.areaOfInterestWidthHeight[0];
    imageDiagnosticsOutputPayload.areaOfInterestWidthHeight[1] =
        this->imageDiagnosticsPayload.areaOfInterestWidthHeight[1];

    if (imageData.centerOfBrightness) {
        imageDiagnosticsOutputPayload.centerOfBrightness[0] = imageData.centerOfBrightness.value()[0] + 0.5;
        imageDiagnosticsOutputPayload.centerOfBrightness[1] = imageData.centerOfBrightness.value()[1] + 0.5;
    }
    if (imageData.brightPixels) { imageDiagnosticsOutputPayload.totalBrightPixels = imageData.brightPixels.value(); }
    if (imageData.coverage) { imageDiagnosticsOutputPayload.coverage = imageData.coverage.value(); }
    this->imageDiagnosticsOutMessage.write(imageDiagnosticsOutputPayload, this->moduleID, currentSimNanos);
}

/*! Get the communication mode
@return ClosedLoopMode enum containing the mode definitions
*/
ClosedLoopMode CielimInterface::getOpNavMode() const {
    return this->opNavMode;
}

/*! Set the communication mode
 * @param ClosedLoopMode enum containing the mode definitions
 */
void CielimInterface::setOpNavMode(ClosedLoopMode mode) {
    this->opNavMode = mode;
}

/*! Set the tcp port number
 * @param std::string port number
 */
void CielimInterface::setPortNumber(std::string port) {
    this->connector.setComPortNumber(port);
}

/*! Get the frame number
 * @return int64_t frame
 */
int64_t CielimInterface::getFrameNumber() const {
    return this->frameNumber;
}

/*! Set the save file path
 * @param std::string path and name to data destination
 */
void CielimInterface::setSaveFile(std::string const &saveProtobufferFile) {
    assert(saveProtobufferFile != "");
    this->protoFilename = saveProtobufferFile;
    this->saveFile = true;
}

/*! Get the save file path
 * @return std::string path and name to data destination
 */
std::string CielimInterface::getSaveFilename() const {
    return this->protoFilename;
}

/*! Manually close the save file for read/write control
 */
void CielimInterface::closeProtobufFile() {
    this->outputStream.close();
}

/*! Toggle live streaming on or off
@param bool on/off
*/
void CielimInterface::setLiveStream(bool liveStreaming) {
    this->liveStream = liveStreaming;
}

/*! Add a celestial body to the interface
@param SpiceBody class containing celestial body information
*/
void CielimInterface::addCelestialBody(SpiceBody const &celestialBodyNames) {
    this->celestialBodiesList.push_back(celestialBodyNames);
}

/*! Get all celestial body added to the interface
@return std::vector<SpiceBody> list of bodies
*/
std::vector<SpiceBody> CielimInterface::getCelestialBodies() const {
    return this->celestialBodiesList;
}

/*! A cleaning method to ensure the message buffers are wiped clean.
 @param data The current sim time in nanoseconds
 @param hint
 */
void message_buffer_deallocate(void* data, void* hint) {
    free(data);
}
