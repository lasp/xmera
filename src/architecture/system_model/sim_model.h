// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _SimModel_HH_
#define _SimModel_HH_

#include <architecture/system_model/sys_process.h>
#include <architecture/utilities/bskLogging.h>

#include <stdint.h>

#include <vector>

//! The top-level container for an entire simulation
class SimModel {
public:
    void selfInitSimulation();                                  //!< Method to initialize all added Tasks
    void resetInitSimulation();                                 //!< Method to reset all added tasks
    void stepUntilStop(uint64_t SimStopTime, int64_t stopPri);  //!< Step simulation until stop time uint64_t reached
    //! Step the processes sharing the earliest resumption time
    /*!
     *  Of all processes in the simulation, one or more will have the earliest
     *  resumption time. (If two processes resume at the same time, both are "earliest".)
     *  This method will allow those processes to resume, but potentially only
     *  partially: only the tasks within those processes whose priority is at least
     *  `stopPri` will be allowed to resume.
     *
     *  @param stopPri
     *    The least priority at which processes' tasks will be resumed
     */
    void singleStepProcesses(int64_t stopPri = -1);
    void addNewProcess(SysProcess* newProc);
    void resetSimulation();  //!< Reset simulation back to zero

    uint64_t getCurrentNanos() const;
    uint64_t getNextTaskTime() const;
    BSKLogger bskLogger;
    std::vector<SysProcess*> processList;  //!< -- List of processes we've created
    std::string SimulationName;            //!< -- Identifier for Sim
    int64_t nextProcPriority = -1;         //!< [-] Priority level for the next process

private:
    uint64_t CurrentNanos = 0;  //!< [ns] Current sim time
    uint64_t NextTaskTime = 0;  //!< [ns] time for the next Task
};

#endif /* _SimModel_H_ */
