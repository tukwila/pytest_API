/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ThreadPoolManager.h
 * @brief  Definition of thread pool manager
 *******************************************************************************
 */

#include <chrono>
#include "boostThread/threadpool.hpp"



#ifndef THREAD_MANAGER
#define THREAD_MANAGER

namespace roadDBCore
{
enum TASK_PRIORITY_E: uint8_t
{
    TASK_PRIORITY_0_E = 0,
    TASK_PRIORITY_1_E = 1,
    TASK_PRIORITY_2_E = 2,
    TASK_PRIORITY_3_E = 3,
    TASK_PRIORITY_4_E = 4,
    TASK_PRIORITY_5_E = 5,
    TASK_PRIORITY_6_E = 6,
    TASK_PRIORITY_7_E = 7,
    TASK_PRIORITY_8_E = 8,
    TASK_PRIORITY_9_E = 9,
    TASK_PRIORITY_MAX_E
};

//using TaskFunction_t = boost::function0<void>;
using TaskFunction_t = std::function<void ()>;
using PrioTaskFuncBase_t = boost::threadpool::prio_task_func;
class RdbPrioTask;
template <typename TASK_TYPE>
class RdbPrioScheduler;

/**
 *******************************************************************************
 * @class RdbPrioTask
 *
 * @brief Prioritized task function object with limited waiting time.
 *
 * This function object wraps a task_func object and binds a priority to it.
 * RdbPrioTask can be compared using the operator < which realises a partial ordering.
 * The wrapped task function is invoked by calling the operator ().
 *******************************************************************************
 */
class RdbPrioTask: public PrioTaskFuncBase_t
{
    using TimePoint_t = std::chrono::system_clock::time_point;
    friend class RdbPrioScheduler<RdbPrioTask>;

public:
    /**
     *******************************************************************************
     * @brief Constructor
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - function  The task's function object.
     *
     *  @param [In]  - priority  The priority of the task.
     *
     *  @param [In]  - waitMilliSeconds  The maximum waiting time before task
     *   execution. 0 means no time limitation. Measured by milliseconds.
     *   If timeout happens, the corresponding task would be deleted.
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    RdbPrioTask(const TaskFunction_t &function = []{},
                     TASK_PRIORITY_E priority = TASK_PRIORITY_0_E,
                     uint32_t waitMilliSeconds = 0):
                     PrioTaskFuncBase_t(priority, function),
                     limitDuration_(waitMilliSeconds)
    {
    }

    /**
    *******************************************************************************
    * @brief getDurationLimit - Get limitation of task waiting time
    *
    *  <1> Parameter Description:
    *
    *  @return limited waiting time of the task
    *
    *  <2> Detailed Description:
    *******************************************************************************
    */
    uint32_t getDurationLimit() const
    {
        return limitDuration_;
    }

private:
    /**
     *******************************************************************************
     * @brief setStartTime - Set task start time when pushed into scheduler queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - startTime Current system time.
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void setStartTime(const TimePoint_t &startTime)
    {
        startTime_ = startTime;
    }

    /**
     *******************************************************************************
     * @brief GetStartTime - Get task start time
     *
     *  <1> Parameter Description:
     *
     *  @return System time when attching the task into scheduler queue
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    const TimePoint_t &getStartTime() const
    {
        return startTime_;
    }

private:
    uint32_t limitDuration_;    // Measured by milliseconds.
    TimePoint_t startTime_;
};

template <typename TASK_TYPE>
using PrioSchedulerBase_t = boost::threadpool::prio_scheduler<TASK_TYPE>;

/**
 *******************************************************************************
 * @class RdbPrioScheduler
 *
 * @brief RdbPrioScheduler which supports prioritized ordering and checking of
 *  waiting time.
 *
 * This container supports a scheduling policy based on task priorities.
 * The task with highest priority will be the first to be removed.
 * It must be possible to compare two tasks using operator<.
 *
 * \param Task A function object which implements the operator() and operator<.
 * operator< must be a partial ordering.
 *
 *******************************************************************************
 */
template <typename TASK_TYPE = RdbPrioTask>
class RdbPrioScheduler: boost::threadpool::prio_scheduler<TASK_TYPE>
{
public:
    /**
     *******************************************************************************
     * @brief push - Adds a new task to the scheduler.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - task The task object.
     *
     *  @return true, if the task could be scheduled and false otherwise.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool push(TASK_TYPE const &task)
    {
        auto &temp = const_cast<TASK_TYPE &>(task);

        temp.setStartTime(std::chrono::system_clock::now());

        return PrioSchedulerBase_t<TASK_TYPE>::push(temp);
    }

    /**
     *******************************************************************************
     * @brief pop - Removes the task which should be executed next.
     *
     *  <1> Parameter Description:
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void pop()
    {
        if (!empty())
        {
            PrioSchedulerBase_t<TASK_TYPE>::pop();
        }
    }

    /**
     *******************************************************************************
     * @brief top - Gets the task which should be executed next.
     *
     *  <1> Parameter Description:
     *
     *  @return The task object to be executed.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    const TASK_TYPE &top()
    {
        while (!empty())
        {
            auto &task = const_cast<TASK_TYPE &>(PrioSchedulerBase_t<TASK_TYPE>::top());

            if (0 == task.getDurationLimit())
            {
                return task;
            }
            else
            {
                auto diff = std::chrono::system_clock::now() - task.getStartTime();
                auto elapsedMS = std::chrono::duration_cast<std::chrono::milliseconds>(diff);

                if (static_cast<uint32_t>(elapsedMS.count()) < task.getDurationLimit())
                {
                    return task;
                }
                else
                {
                    pop();
                    COM_LOG_WARN << "A task has been removed from pending list due to timeout!";
                }
            }
        }

        COM_LOG_INFO << "No pending task, dummy task would be returned.";

        return dummyTask;
    }

    /**
     *******************************************************************************
     * @brief size -  Gets the current number of tasks in the scheduler.
     *  Prefer empty() to size() == 0 to check if the scheduler is empty.
     *
     *  <1> Parameter Description:
     *
     *  @return The number of tasks.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    size_t size() const
    {
        return PrioSchedulerBase_t<TASK_TYPE>::size();
    }

    /**
     *******************************************************************************
     * @brief empty -  Checks if the scheduler is empty.
     *  Is more efficient than size() == 0.
     *
     *  <1> Parameter Description:
     *
     *  @return true if the scheduler contains no tasks, false otherwise.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool empty() const
    {
        return PrioSchedulerBase_t<TASK_TYPE>::empty();
    }

    /**
     *******************************************************************************
     * @brief clear - Removes all tasks from the scheduler.
     *
     *  <1> Parameter Description:
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void clear()
    {
        PrioSchedulerBase_t<TASK_TYPE>::clear();
    }

private:
    static TASK_TYPE dummyTask;
};

template <typename TASK_TYPE>
TASK_TYPE RdbPrioScheduler<TASK_TYPE>::dummyTask;

/**
 *******************************************************************************
 * @class RdbPrioThreadPool_t
 *
 * @brief Thread pool for prioritized task with limited waiting time.
 *
 * The pool's tasks are prioritized RdbPrioTask functors.
 *******************************************************************************
 */
using BoostPrioThreadPool_t = boost::threadpool::thread_pool<
                              RdbPrioTask,
                              RdbPrioScheduler,
                              boost::threadpool::static_size,
                              boost::threadpool::resize_controller,
                              boost::threadpool::wait_for_all_tasks>;

/**
 *******************************************************************************
 * @class RdbPrioThreadPool
 *
 * @brief Encapsulation of thread pool class BoostPrioThreadPool_t defined above.
 *
 * Please refer to class thread_pool defined in
 * \core\common\thirdParty\boostThread\threadpool\pool.hpp for more details.
 * Note: Need to add boost thread lib and pthread lib for linking in CMakeList.
 * For example:
 * FIND_PACKAGE(Boost REQUIRED system filesystem thread)
 * include_directories(${Boost_INCLUDE_DIR})
 * target_link_libraries(xxx
 *                       ...
 *                       pthread
 *                       ${Boost_LIBRARIES})
 *******************************************************************************
 */
 class RdbPrioThreadPool
 {
    using size_t = uint32_t;

 public:
    /**
     *******************************************************************************
     * @brief Constructor -  Construct thread pool
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - threadsNum The pool is immediately resized to set the
     *   specified number of threads.
     *
     *  @return
     *
     *  <2> Detailed Description: The pool's actual number threads depends on the
     *      SizePolicy.
     *******************************************************************************
     */
    RdbPrioThreadPool(size_t threadsNum): threadPool(threadsNum) {}

    /**
     *******************************************************************************
     * @brief size -  Gets the number of threads in the pool.
     *
     *  <1> Parameter Description:
     *
     *  @return The number of threads.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    size_t size() const
    {
        return threadPool.size();
    }

    /**
     *******************************************************************************
     * @brief schedule -  Schedules a task for asynchronous execution. The task will
     *  be executed once only.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - task The task function object. It should not throw execeptions.
     *
     *  @return true, if the task could be scheduled and false otherwise.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool schedule(const RdbPrioTask &task)
    {
        return threadPool.schedule(task);
    }

    /**
     *******************************************************************************
     * @brief active -  Returns the number of tasks which are currently executed.
     *
     *  <1> Parameter Description:
     *
     *  @return The number of active tasks.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    size_t active() const
    {
        return threadPool.active();
    }

    /**
     *******************************************************************************
     * @brief pending -  Returns the number of tasks which are ready for execution.
     *  be executed once only.
     *
     *  <1> Parameter Description:
     *
     *  @return The number of pending tasks.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    size_t pending() const
    {
        return threadPool.pending();
    }

    /**
     *******************************************************************************
     * @brief clear -  Removes all pending tasks from the pool's scheduler.
     *
     *  <1> Parameter Description:
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void clear()
    {
        threadPool.clear();
    }

    /**
     *******************************************************************************
     * @brief empty -  Indicates that there are no tasks pending.
     *  This function is more efficient that the check 'pending() == 0'.
     *
     *  <1> Parameter Description:
     *
     *  @return true if there are no tasks ready for execution.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool empty() const
    {
        return threadPool.empty();
    }

    /**
     *******************************************************************************
     * @brief wait - The current thread of execution is blocked until the sum of all
     *  active and pending tasks is equal or less than a given threshold.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - taskThreshold The maximum number of tasks in pool and scheduler.
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void wait(size_t taskThreshold = 0) const
    {
        threadPool.wait(taskThreshold);
    }

#if 0
    /**
     *******************************************************************************
     * @brief schedule -  The current thread of execution is blocked until the
     *  maximum waiting time is met or the sum of all active and pending tasks is
     *  equal or less than a given threshold.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - waitMilliSeconds The maximum waiting time when function returns.
     *
     *  @param [In]  - taskThreshold The maximum number of tasks in pool and scheduler.
     *
     *  @return true if the task sum is equal or less than the threshold, false otherwise.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool wait(uint32_t waitMilliSeconds, size_t taskThreshold = 0) const
    {
        boost::xtime resTime;

        if (boost::TIME_UTC_ != boost::xtime_get(resTime, boost::TIME_UTC_))
        {
            return false;
        }

        resTime.sec = static_cast<boost::xtime::xtime_sec_t>(waitMilliSeconds / 1000);
        resTime.nsec = static_cast<boost::xtime::xtime_nsec_t>((waitMilliSeconds % 1000) * 1000000);

        return threadPool.wait(resTime, taskThreshold);
    }
#endif

 private:
    BoostPrioThreadPool_t threadPool;
 };




}// namespace roadDBCore




#endif // THREAD_MANAGER

