// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module centerOfBrightness
%{
   #include <memory>
   #include "centerOfBrightness.h"
   #include "imageReader/imageReaderInterface.h"
   #include "imageReader/imageReaderFromFile.h"
   #include "imageReader/imageReaderFromMessage.h"
%}

%include <stdint.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_array.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <std_shared_ptr.i>

%shared_ptr(ImageReaderInterface)
%shared_ptr(ImageReaderFromFile)
%shared_ptr(ImageReaderFromMessage)

%include "imageReader/imageReaderInterface.h"
%include "imageReader/imageReaderFromFile.h"
%include "imageReader/imageReaderFromMessage.h"

%include "centerOfBrightness.h"

%include <architecture/msgPayloadDef/CameraImageMsgPayload.h>
%include <architecture/msgPayloadDef/RegionOfInterestMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavCOBMsgPayload.h>
%include <architecture/msgPayloadDef/CenterOfBrightnessDiagnosticMsgPayload.h>
