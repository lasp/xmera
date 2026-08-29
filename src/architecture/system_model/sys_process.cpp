// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sys_process.h"

#include <architecture/system_model/sim_instant.h>

#include <iostream>
#include <utility>

std::vector<SysModelTask*>::iterator SysProcess::getNextTask() {
    auto nextTaskIt = this->processTasks.begin();

    for (auto it = this->processTasks.begin(); it != this->processTasks.end(); it++) {
        auto currentUpdateTime = SimInstant::atNanos((*it)->getNextStartTime()).atPriority((*it)->priority);
        auto earliestUpdateTime = SimInstant::atNanos((*nextTaskIt)->getNextStartTime()).atPriority((*nextTaskIt)->priority);

        if (currentUpdateTime < earliestUpdateTime) { nextTaskIt = it; }
    }

    return nextTaskIt;
}

SysModelTask &SysProcess::addTask(uint64_t updatePeriodNanos, uint64_t firstUpdateNanos, int32_t priority) {
    auto &task = this->allocatedTasks.emplace_back(std::make_unique<SysModelTask>(
        updatePeriodNanos,
        firstUpdateNanos,
        priority
    ));

    this->scheduleTask(task.get());

    return *task.get();
}

void SysProcess::scheduleTask(SysModelTask* task) {
    // Find the index separating higher priority tasks from lower priority tasks.
    // Modules will be reset in this order, and modules that update at the same
    // time will be tie-broken by this order.
    auto it = this->processTasks.begin();
    for (; it != this->processTasks.end(); ++it) {
        if (task->priority > (*it)->priority) { break; }
    }

    // Insert the module at this index. (It's okay if it's the end() iterator.)
    this->processTasks.insert(it, task);
}

bool SysProcess::changeTaskPeriod(std::string const &taskName, uint64_t newPeriod) {
    for (auto &entry : this->processTasks) {
        if (entry->TaskName != taskName) { continue; }

        entry->setPeriod(newPeriod);

        // Determine the new next task time. (If the next task was this one, it
        // might now be some other task. Worse, it might *still* be this one!)
        this->nextTaskTime = (*this->getNextTask())->getNextStartTime();

        return true;
    }

    return false;
}
