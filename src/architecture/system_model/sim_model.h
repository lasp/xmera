// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _SimModel_HH_
#define _SimModel_HH_

#include <architecture/system_model/sys_process.h>

#include <stdint.h>

#include <vector>

//! The top-level container for an entire simulation
class SimModel {
public:
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

    //! Get an immutable view on the list of processes in this simulation
    /*!
     *  Note that it is the list of processes that is immutable here, not the
     *  processes themselves. It is perfectly legal to mutate one of the processes
     *  obtained from this method, so long as no other protocol of use is violated.
     */
    std::vector<SysProcess*> const &getProcesses() const {
        return this->processList;
    }

private:
    uint64_t CurrentNanos    = 0;   //!< [ns] Current sim time
    uint64_t NextTaskTime    = 0;   //!< [ns] time for the next Task
    int64_t nextProcPriority = -1;  //!< [-] Priority level for the next process

    std::vector<SysProcess*> processList;  //!< -- List of processes we've created
};

#endif /* _SimModel_H_ */
