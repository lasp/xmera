// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef COMMON_UTILS_SEMAPHORE_H
#define COMMON_UTILS_SEMAPHORE_H
// https://riptutorial.com/cplusplus/example/30142/semaphore-cplusplus-11

#include <condition_variable>
#include <mutex>

/*! Basilisk semaphore class */
class BSKSemaphore {
    std::mutex mutex;
    std::condition_variable cv;
    size_t count;

   public:
    /*! method description */
    BSKSemaphore(int count_in = 0) : count(count_in) {}

    /*! release the lock */
    inline void release() {
        {
            std::unique_lock<std::mutex> lock(mutex);
            ++count;
            // notify the waiting thread
        }
        cv.notify_one();
    }

    /*! aquire the lock */
    inline void acquire() {
        std::unique_lock<std::mutex> lock(mutex);
        while (count == 0) {
            // wait on the mutex until notify is called
            cv.wait(lock);
        }
        --count;
    }
};

#endif
