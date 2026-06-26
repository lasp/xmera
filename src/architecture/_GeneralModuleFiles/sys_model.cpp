// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "architecture/_GeneralModuleFiles/sys_model.h"

#include <atomic>

//! The module ID to be assigned to the next module to be constructed
static std::atomic<int64_t> nextModuleID = 1;

SysModel::SysModel() : RNGSeed{0x1bad'cad1}, moduleID(nextModuleID++) {}

SysModel::SysModel(SysModel const &other)
    : modelTag{other.modelTag}, RNGSeed{other.RNGSeed}, moduleID{nextModuleID++} {}
