/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   BarrierManager.h
 * @brief  Definition of boost barrier manager
 *******************************************************************************
 */


#include <map>
#include <mutex>
#include <memory>
#include <boost/thread/barrier.hpp>

#ifndef BARRIER_MANAGER_H
#define BARRIER_MANAGER_H

namespace roadDBCore
{
/*
const char *START_BARRIER      = "start";
const char *END_BARRIER        = "end";
const char *RESET_BARRIER      = "reset";
const char *INIT_BARRIER       = "init";
const char *QUICK_INIT_BARRIER = "quickInit";
*/

enum BARRIER_KEY_E
{
    BARRIER_KEY_START_E,
    BARRIER_KEY_END_E,
    BARRIER_KEY_RESET_E,
    BARRIER_KEY_MAX_E
};

/**
 *******************************************************************************
 * @class BarrierManager
 *
 * @brief Can be used to sync up multi threads which have been register the same
 *  barrier, in order to make sure all the threads related to the same barrier
 *  are ready to handle something.
 *******************************************************************************
 */
class BarrierManager
{
public:
    BarrierManager() {}

    ~BarrierManager() {}

    using Barrier_t = boost::barrier;

    /*
     *  BarrierKey_t can be implemented as BARRIER_KEY_E (barrier type),
     *  uint32_t (barrier ID) or std::string (barrier name) .
     */
    using BarrierKey_t = uint32_t; // std::string

    using BarrierValue_t = std::pair<uint32_t, std::shared_ptr<Barrier_t>>;

    /**
     *******************************************************************************
     * @brief registerx - Register the barrier in order to sync up with other threads
     *  which have been register the same barrier too.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - barrierKey   Used to uniquely identify each barrier element
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool registerx(BarrierKey_t barrierKey)
    {
        std::lock_guard<std::mutex> lg(mutex_);
        auto &barrierItem = barrierMap_[barrierKey];

        if (barrierItem.second)
        {
            COM_LOG_ERROR << "The barrier "
                          << barrierKey
                          << " has been created, unable to register for it! count: "
                          << barrierItem.first;

            return false;
        }
        else
        {
            ++barrierItem.first;
            COM_LOG_INFO << "Register for the barrier "
                         << barrierKey
                         << ", count: "
                         << barrierItem.first;

            return true;
        }
    }

    /**
     *******************************************************************************
     * @brief wait - This method is used to sync up with other threads which have
     *  been register the same barrier too.
     *  The calling would cause suspension of the thread until all the register
     *  threads have been ready.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - barrierKey   Used to uniquely identify each barrier element
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool wait(BarrierKey_t barrierKey)
    {
        std::shared_ptr<Barrier_t> spBarrier;

        {
            std::lock_guard<std::mutex> lg(mutex_);
            auto &barrierItem = barrierMap_[barrierKey];

            if (0 == barrierItem.first)
            {
                COM_LOG_ERROR << "None of the thread has been register for the barrier "
                              << barrierKey;

                return false;
            }
            else
            {
                if (!barrierItem.second)
                {
                    barrierItem.second = std::make_shared<Barrier_t>(barrierItem.first);
                    COM_LOG_INFO << "Create the barrier "
                                 << barrierKey
                                 << ", count: "
                                 << barrierItem.first;
                }

                spBarrier = barrierItem.second;
            }
        }

        if (spBarrier)
        {
            spBarrier->wait();
        }
        else
        {
            COM_LOG_ERROR << "Failed to create the barrier " << barrierKey;

            return false;
        }

        return true;
    }

    /**
     *******************************************************************************
     * @brief remove - Remove the barrier element specified by barrier name
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - barrierKey   Used to uniquely identify each barrier element
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool remove(BarrierKey_t barrierKey)
    {
        bool status = false;
        std::lock_guard<std::mutex> lg(mutex_);

        if (barrierMap_.find(barrierKey) != barrierMap_.end())
        {
            barrierMap_.erase(barrierKey);
            status = true;
        }

        return status;
    }

    /**
     *******************************************************************************
     * @brief clear - Empty barrier map.
     *
     *  <1> Parameter Description:
     *
     *  @return The number of event has been removed.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void clear()
    {
        std::lock_guard<std::mutex> lg(mutex_);

        barrierMap_.clear();
    }

    /**
     *******************************************************************************
     * @brief empty - Test whether the barrier map is empty
     *
     *  <1> Parameter Description:
     *
     *  @return True if empty, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool empty() const
    {
        std::lock_guard<std::mutex> lg(mutex_);

        return barrierMap_.empty();
    }

    /**
     *******************************************************************************
     * @brief count - Get the count number of specified barrier
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - barrierKey   Used to uniquely identify each barrier element
     *
     *  @return The count number of specified barrier
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    uint32_t count(BarrierKey_t barrierKey) const
    {
        std::lock_guard<std::mutex> lg(mutex_);

        auto iter = barrierMap_.find(barrierKey);

        if (iter != barrierMap_.end())
        {
            return iter->first;
        }
        else
        {
            return 0;
        }
    }

    /**
     * No copy constructor.
     */
    BarrierManager(const BarrierManager&) = delete;

    /**
     * No assignment.
     */
    BarrierManager &operator=(const BarrierManager&) = delete;

private:
    std::map<BarrierKey_t, BarrierValue_t> barrierMap_;
    mutable std::mutex mutex_;
};

}

#endif
