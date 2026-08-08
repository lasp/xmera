// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sys_process.h"

#include <architecture/system_model/sim_instant.h>

#include <iostream>
#include <utility>

std::vector<ModelScheduleEntry>::iterator SysProcess::getNextTask() {
    auto nextTaskIt = this->processTasks.begin();

    for (auto it = this->processTasks.begin(); it != this->processTasks.end(); it++) {
        auto currentUpdateTime = SimInstant::atNanos(it->NextTaskStart).atPriority(it->taskPriority);
        auto earliestUpdateTime = SimInstant::atNanos(nextTaskIt->NextTaskStart).atPriority(nextTaskIt->taskPriority);

        if (currentUpdateTime < earliestUpdateTime) { nextTaskIt = it; }
    }

    return nextTaskIt;
}

void SysProcess::singleStepNextTask(uint64_t nextSimNanos) {
    if (this->processTasks.empty() || this->nextTaskTime > nextSimNanos) { return; }

    // Update the next task, and record when it wants to be updated again
    auto nextTaskIt = this->getNextTask();
    nextTaskIt->TaskPtr->executeModels(nextSimNanos);
    nextTaskIt->NextTaskStart = nextTaskIt->TaskPtr->getNextStartTime();

    // Record when the next task will update
    this->nextTaskTime = this->getNextTask()->NextTaskStart;
}

SysModelTask &SysProcess::addTask(uint64_t updatePeriodNanos, uint64_t firstUpdateNanos, int32_t priority) {
    auto &task = this->allocatedTasks.emplace_back(std::make_unique<SysModelTask>(updatePeriodNanos, firstUpdateNanos));

    this->scheduleTask({
        .NextTaskStart = task->getNextStartTime(),
        .TaskUpdatePeriod = task->getTaskPeriod(),
        .taskPriority = priority,
        .TaskPtr = task.get(),
    });

    return *task.get();
}

void SysProcess::scheduleTask(ModelScheduleEntry const &scheduleEntry) {
    // Find the index separating higher priority tasks from lower priority tasks.
    // Modules will be reset in this order, and modules that update at the same
    // time will be tie-broken by this order.
    auto it = this->processTasks.begin();
    for (; it != this->processTasks.end(); ++it) {
        if (scheduleEntry.taskPriority > it->taskPriority) { break; }
    }

    // Insert the module at this index. (It's okay if it's the end() iterator.)
    this->processTasks.insert(it, scheduleEntry);
}

bool SysProcess::changeTaskPeriod(std::string const &taskName, uint64_t newPeriod) {
    for (auto &entry : this->processTasks) {
        if (entry.TaskPtr->TaskName != taskName) { continue; }

        entry.TaskPtr->setPeriod(newPeriod);
        entry.NextTaskStart = entry.TaskPtr->getNextStartTime();
        entry.TaskUpdatePeriod = entry.TaskPtr->getTaskPeriod();

        // Determine the new next task time. (If the next task was this one, it
        // might now be some other task. Worse, it might *still* be this one!)
        this->nextTaskTime = this->getNextTask()->NextTaskStart;

        return true;
    }

    return false;
}
