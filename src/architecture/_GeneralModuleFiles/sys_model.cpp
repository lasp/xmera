#include "architecture/_GeneralModuleFiles/sys_model.h"

static int64_t nextModuleID = 1;

SysModel::SysModel() : moduleID(nextModuleID++) {}

SysModel::SysModel(const SysModel& obj) : modelTag{obj.modelTag}, RNGSeed{obj.RNGSeed}, moduleID{nextModuleID++} {}
