/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ThreadSafeMap.h
 * @brief  Definition of data map manager
 *******************************************************************************
 */


#include <map>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <chrono>

#ifndef THREAD_SAFE_MAP_H
#define THREAD_SAFE_MAP_H

namespace roadDBCore
{

/**
 *******************************************************************************
 * @class ThreadSafeMap
 *
 * @brief Support sharing <key, value> data objects among threads.
 *  1. Data producer: Insert <key, value> data object into map.
 *  2. Data consumer: Get and remove data object from the map according to input
 *     key.
 *******************************************************************************
 */

template<typename KEY_TYPE, typename VALUE_TYPE>
class ThreadSafeMap
{
public:
    using ElementSize_t = typename std::map<KEY_TYPE, VALUE_TYPE>::size_type;

    ThreadSafeMap() {}

    ~ThreadSafeMap() {}

    /**
     *******************************************************************************
     * @brief push - Save the <key, value> data object into data map.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - key   Used to uniquely identify each map element
     *
     *  @param [In]  - value   Mapped value of each map element
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void push(const KEY_TYPE &key, const VALUE_TYPE &value)
    {
        std::lock_guard<std::mutex> lg(mutex_);
        dataMap_.emplace(key, value);
        cond_.notify_one();
    }

    /**
     *******************************************************************************
     * @brief pull - Get element value from data map according to specified key and
     *  then remove the element.
     *  If the element doesn't exist, then waiting until it's ready.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - key   Used to uniquely identify each map element
     *
     *  @param [InOut]  - value   Mapped value of each map element
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool pull(const KEY_TYPE &key, VALUE_TYPE &value)
    {
        bool status = false;
        typename std::map<KEY_TYPE, VALUE_TYPE>::iterator iter;
        std::unique_lock<std::mutex> ul(mutex_);

        cond_.wait(ul,
            [this, &iter, &key] {return (iter = dataMap_.find(key)) != dataMap_.end();});
        value = iter->second;
        dataMap_.erase(iter);

        return true;
    }

    /**
     *******************************************************************************
     * @brief pull -  Get element value from data map according to specified key and
     *  then remove the element with specified maximum waiting time.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - key   Used to uniquely identify each map element
     *
     *  @param [InOut]  - value   Mapped value of each map element
     *
     *  @param [In]  - waitMilliSeconds   Maximum waiting time measured by millisecond
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool pull(const KEY_TYPE &key, VALUE_TYPE &value, uint32_t waitMilliSeconds)
    {
        bool status = false;
        typename std::map<KEY_TYPE, VALUE_TYPE>::iterator iter;
        std::unique_lock<std::mutex> ul(mutex_);

        if (waitMilliSeconds)
        {
            std::chrono::milliseconds waitTime(waitMilliSeconds);

            cond_.wait_for(ul, waitTime,
                [this, &iter, &key] {return (iter = dataMap_.find(key)) != dataMap_.end();});
        }

        if (iter != dataMap_.end())
        {
            value = iter->second;
            dataMap_.erase(iter);
            status = true;
        }

        return status;
    }

    /**
     *******************************************************************************
     * @brief peek - Just get the first data object from data map without removing.
     *  If the element map is empty, then return flase without waiting.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - key   Used to uniquely identify each map element
     *
     *  @param [InOut]  - value   Mapped value of each map element
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool peek(const KEY_TYPE &key, VALUE_TYPE &value) const
    {
        bool status = false;
        std::lock_guard<std::mutex> lg(mutex_);

        auto iter = dataMap_.find(key);

        if (iter != dataMap_.end())
        {
            value = iter->second;
            status = true;
        }

        return status;
    }

    /**
     *******************************************************************************
     * @brief remove - Remove the map element specified by input key
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - key   Used to uniquely identify the map element to be deleted
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool remove(const KEY_TYPE &key)
    {
        bool status = false;
        std::lock_guard<std::mutex> lg(mutex_);

        if (dataMap_.find(key) != dataMap_.end())
        {
            dataMap_.erase(key);
            status = true;
        }

        return status;
    }

    /**
     *******************************************************************************
     * @brief clear - Empty data map.
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
        std::lock_guard<std::mutex> lg(mutex_);

        dataMap_.clear();
    }

    /**
     *******************************************************************************
     * @brief empty - Test whether the data map is empty
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

        return dataMap_.empty();
    }

    /**
     *******************************************************************************
     * @brief size - Get the size of the data map
     *
     *  <1> Parameter Description:
     *
     *  @return Number of data elements in data map
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    ElementSize_t size() const
    {
        std::lock_guard<std::mutex> lg(mutex_);

        return dataMap_.size();
    }

    /**
     * No copy constructor.
     */
    ThreadSafeMap(const ThreadSafeMap&) = delete;

    /**
     * No assignment.
     */
    ThreadSafeMap &operator=(const ThreadSafeMap&) = delete;

private:
    std::map<KEY_TYPE, VALUE_TYPE> dataMap_;
    mutable std::mutex mutex_;
    std::condition_variable cond_;
};

}

#endif
