// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "architecture/_GeneralModuleFiles/sys_model.h"

static int64_t nextModuleID = 1;

SysModel::SysModel() : moduleID(nextModuleID++) {}

SysModel::SysModel(const SysModel& obj) : modelTag{obj.modelTag}, RNGSeed{obj.RNGSeed}, moduleID{nextModuleID++} {}
