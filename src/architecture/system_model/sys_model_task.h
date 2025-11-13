#ifndef _SysModelTask_HH_
#define _SysModelTask_HH_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/utilities/bskLogging.h>
#include <stdint.h>
#include <vector>

//! Structure used to pair a model and its requested priority
typedef struct {
    int32_t CurrentModelPriority;  //!< The current model priority. Higher goes first
    SysModel* ModelPtr;            //!< The model associated with this priority
} ModelPriorityPair;

//! Class used to group a set of models into one "Task" of execution
class SysModelTask {
   public:
    SysModelTask() = default;
    explicit SysModelTask(uint64_t InputPeriod, uint64_t FirstStartTime = 0);  //!< class method
    ~SysModelTask() = default;
    void addModel(SysModel* NewModel, int32_t Priority = -1);
    void selfInitTaskList() const;
    // void CrossInitTaskList();
    void executeModels(uint64_t CurrentSimTime);
    void resetModels(uint64_t CurrentSimTime);
    void reset() { this->NextStartTime = this->FirstTaskTime; }  //!< Resets the task
    void enable() { this->taskActive = true; }                   //!< Enables the task.  Great comment huh?
    void disable() { this->taskActive = false; }                 //!< Disables the task.  I know.
    void setPeriod(uint64_t newPeriod);
    void setParentProc(std::string const& parent) {
        this->parentProc = parent;
    }  //!< Allows the system to move task to a different process
    uint64_t getNextStartTime() const;
    uint64_t getNextPickupTime() const;
    uint64_t getTaskPeriod() const;
    uint64_t getFirstTaskTime() const;

    std::vector<ModelPriorityPair> TaskModels{};  //!< -- Array that has pointers to all task sysModels
    std::string TaskName{};                       //!< -- Identifier for Task
    std::string parentProc;                       //!< -- Process that calls this task
    bool taskActive = true;                       //!< -- Flag indicating whether the Task has been disabled
    BSKLogger bskLogger;                          //!< -- BSK Logging
   private:
    uint64_t NextStartTime = 0;   //!< [ns] Next time to start task
    uint64_t NextPickupTime = 0;  //!< [ns] Next time read Task outputs
    uint64_t TaskPeriod = 100;    //!< [ns] Cycle rate for Task
    uint64_t FirstTaskTime =
        0;  //!< [ns] Time to start Task for first time.  After this time the normal periodic updates resume.
};

#endif /* _SysModelTask_H_ */
