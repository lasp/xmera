// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sys_model_task.h"

inline uint64_t SysModelTask::projectToCurrentSchedule(uint64_t time) {
    // Judge everything relative to the origin, `firstUpdateNanos`.
    time -= this->firstUpdateNanos;

    // Truncate `time` to the nearest mutiple of `updatePeriodNanos`.
    time = (time / this->updatePeriodNanos) * this->updatePeriodNanos;

    // Re-add the offset against `firstUpdateNanos`.
    time += this->firstUpdateNanos;

    return time;
}

void SysModelTask::executeModels(uint64_t nextSimNanos) {
    this->nextUpdateNanos += this->updatePeriodNanos;

    if (!this->taskActive) { return; }

    for (auto &modelPair : this->TaskModels) { modelPair.ModelPtr->updateState(nextSimNanos); }
}

void SysModelTask::addModel(SysModel* module, int32_t priority) {
    // Find the index separating lower priorities from higher priorities.
    auto it = this->TaskModels.begin();
    for (; it != this->TaskModels.end(); ++it) {
        if (priority > it->CurrentModelPriority) { break; }
    }

    // Insert the module at this index. (It's okay if it's the end() iterator.)
    this->TaskModels.insert(it, {.CurrentModelPriority = priority, .ModelPtr = module});
}

void SysModelTask::setPeriod(uint64_t updatePeriodNanos) {
    if (this->nextUpdateNanos > this->firstUpdateNanos) {
        uint64_t lastUpdateTime = this->nextUpdateNanos - this->updatePeriodNanos;

        this->updatePeriodNanos = updatePeriodNanos;
        this->nextUpdateNanos = this->projectToCurrentSchedule(this->nextUpdateNanos);
        if (this->nextUpdateNanos <= lastUpdateTime) { this->nextUpdateNanos += updatePeriodNanos; }
    } else {
        // If we haven't yet performed an update, our next update time is *still*
        // our first update time.
        this->updatePeriodNanos = updatePeriodNanos;
    }
}
