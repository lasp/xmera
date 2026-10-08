%module fpgaImagePipeline
%{
   #include "fpgaImagePipeline.h"
%}

%include <stdint.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <attribute.i>
%attribute(FpgaImagePipeline, uint32_t, imageWidth, getImageWidth, setImageWidth)
%attribute(FpgaImagePipeline, uint32_t, imageHeight, getImageHeight, setImageHeight)
%attribute(FpgaImagePipeline, uint8_t, kernelSize, getKernelSize, setKernelSize)
%attribute(FpgaImagePipeline, uint16_t, threshold, getThreshold, setThreshold)
%attribute(FpgaImagePipeline, uint32_t, roiRegionSize, getRoiRegionSize, setRoiRegionSize)
%attribute(FpgaImagePipeline, uint16_t, calibRegA, getCalibRegA, setCalibRegA)
%attribute(FpgaImagePipeline, uint16_t, calibRegB, getCalibRegB, setCalibRegB)
%attribute(FpgaImagePipeline, uint16_t, calibRegC, getCalibRegC, setCalibRegC)
%attribute(FpgaImagePipeline, uint16_t, calibRegD, getCalibRegD, setCalibRegD)
%attribute(FpgaImagePipeline, bool, calibEnabled, getCalibEnabled, setCalibEnabled)
%attribute(FpgaImagePipeline, bool, saveImages, getSaveImages, setSaveImages)
%attributestring(FpgaImagePipeline, std::string, saveDir, getSaveDir, setSaveDir)

%include "fpgaImagePipeline.h"

%include <architecture/msgPayloadDef/CameraImageMsgPayload.h>
%include <architecture/msgPayloadDef/FpgaRawImageMsgPayload.h>
%include <architecture/msgPayloadDef/FpgaThreshImageMsgPayload.h>
%include <architecture/msgPayloadDef/FpgaRowColSumMsgPayload.h>
STRUCTASLIST(FpgaRoiEntry)
%include <architecture/msgPayloadDef/FpgaBinsMsgPayload.h>
%include <architecture/msgPayloadDef/FpgaPipelineConfigMsgPayload.h>
