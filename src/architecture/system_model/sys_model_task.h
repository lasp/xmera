// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMAheader_sys_model_task
#define XMAheader_sys_model_task

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/utilities/bskLogging.h>
#include <stdint.h>
#include <vector>

//! Structure used to pair a model and its requested priority
struct ModelPriorityPair {
    int32_t CurrentModelPriority;  //!< The current model priority. Higher goes first
    SysModel* ModelPtr;            //!< The model associated with this priority
};

//! Class used to group a set of models into one "Task" of execution
class SysModelTask {
public:
    SysModelTask() = default;
    ~SysModelTask() = default;

    explicit SysModelTask(uint64_t InputPeriod, uint64_t FirstStartTime = 0);

    void addModel(SysModel* NewModel, int32_t Priority = -1);
    void executeModels(uint64_t CurrentSimTime);
    void resetModels(uint64_t CurrentSimTime);
    void setPeriod(uint64_t newPeriod);

    void reset() {
        this->NextStartTime = this->FirstTaskTime;
    }

    void enable() {
        this->taskActive = true;
    }

    void disable() {
        this->taskActive = false;
    }

    uint64_t getNextStartTime() const;
    uint64_t getTaskPeriod() const;
    uint64_t getFirstTaskTime() const;

private:
    uint64_t projectToCurrentSchedule(uint64_t time);

public:
    std::vector<ModelPriorityPair> TaskModels{};  //!< -- Array that has pointers to all task sysModels
    std::string TaskName{};                       //!< -- Identifier for Task
    bool taskActive = true;                       //!< -- Flag indicating whether the Task has been disabled
    BSKLogger bskLogger{};                        //!< -- BSK Logging

private:
    uint64_t NextStartTime = 0;  //!< [ns] Next time to start task
    uint64_t TaskPeriod = 100;   //!< [ns] Cycle rate for Task
    uint64_t FirstTaskTime = 0;  //!< [ns] Time to start Task for first time.
                                 //!<      After this time the normal periodic updates resume.
};

#endif
