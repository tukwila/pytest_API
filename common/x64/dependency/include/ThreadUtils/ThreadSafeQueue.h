/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018-2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ThreadSafeQueue.h
 * @brief  Definition of thread safe queue
 *******************************************************************************
 */


#include <deque>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <chrono>

#ifndef THREAD_SAFE_QUEUE_H
#define THREAD_SAFE_QUEUE_H

namespace roadDBCore
{

/**
 *******************************************************************************
 * @class ThreadSafeQueue
 *
 * @brief Support sharing data objects among threads.
 *  1. Data producer: Save data object into queue.
 *  2. Data consumer: Retrieve data object from the queue and process it.
 *  Note: For simple data object type, the template TYPE can be the real object
 *  type;
 *        For compound data object type, suggest setting template TYPE to
 *  std::shared_ptr<data object type>.
 *******************************************************************************
 */

template<typename TYPE>
class ThreadSafeQueue
{
public:
    using ElementSize_t = typename std::deque<TYPE>::size_type;

    ThreadSafeQueue() {}

    ~ThreadSafeQueue() {}

    /**
     *******************************************************************************
     * @brief push - Save the data object into data queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - object   Data object to be saved
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void push(const TYPE &object)
    {
        std::lock_guard<std::mutex> lg(mutex_);
        dataQueue_.push_back(object);
        cond_.notify_one();
    }

    /**
     *******************************************************************************
     * @brief pull - Retrieve (get and remove) data object from data queue.
     *  If the data queue is empty, then waiting until any data object is ready.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - object   Data object to be retrived
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool pull(TYPE &object)
    {
        bool status = false;
        std::unique_lock<std::mutex> ul(mutex_);

        cond_.wait(ul, [this] {return !dataQueue_.empty();});
        object = dataQueue_.front();
        dataQueue_.pop_front();

        return true;
    }

    /**
     *******************************************************************************
     * @brief pull - Retrieve (get and remove) data object from data queue
     *  with specified maximum waiting time.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - object   Data object to be retrived
     *
     *  @param [In]  - waitMilliSeconds   Maximum waiting time measured by millisecond
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool pull(TYPE &object, uint32_t waitMilliSeconds)
    {
        bool status = false;
        std::unique_lock<std::mutex> ul(mutex_);

        if (waitMilliSeconds)
        {
            std::chrono::milliseconds waitTime(waitMilliSeconds);

            cond_.wait_for(ul, waitTime, [this] {return !dataQueue_.empty();});
        }

        if (!dataQueue_.empty())
        {
            object = dataQueue_.front();
            dataQueue_.pop_front();
            status = true;
        }

        return status;
    }

    /**
     *******************************************************************************
     * @brief pull - Retrieve (get and remove) data object from data queue.
     *  If the data queue is empty, then waiting until any data object is ready.
     *
     *  <1> Parameter Description:
     *
     *  @return data object.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    TYPE pull()
    {
        std::unique_lock<std::mutex> ul(mutex_);

        cond_.wait(ul, [this] {return !dataQueue_.empty();});

        TYPE object = dataQueue_.front();

        dataQueue_.pop_front();

        return object;
    }

    /**
     *******************************************************************************
     * @brief peek - Just get the first data object from data queue without removing.
     *  If the data queue is empty, then return flase without waiting.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - object   Data object to be retrived
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool peek(TYPE &object) const
    {
        bool status = false;
        std::lock_guard<std::mutex> lg(mutex_);

        if (!dataQueue_.empty())
        {
            object = dataQueue_.front();
            status = true;
        }

        return status;
    }

    /**
     *******************************************************************************
     * @brief peekAndWait - Just peek the first data object from data queue without removing.
     *  with specified maximum waiting time.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - object   Data object to be retrived
     *
     *  @param [In]  - waitMilliSeconds   Maximum waiting time measured by millisecond
     *
     *  @return True if success, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool peekAndWait(TYPE &object, uint32_t waitMilliSeconds)
    {
        bool status = false;
        std::unique_lock<std::mutex> ul(mutex_);

        if (waitMilliSeconds)
        {
            std::chrono::milliseconds waitTime(waitMilliSeconds);

            cond_.wait_for(ul, waitTime, [this] {return !dataQueue_.empty();});
        }

        if (!dataQueue_.empty())
        {
            object = dataQueue_.front();
            status = true;
        }

        return status;
    }
    /**
     *******************************************************************************
     * @brief pop - Remove the first data object from the data queue
     *
     *  <1> Parameter Description:
     *
     *  @return True if removed a data object, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool pop()
    {
        bool status = false;
        std::lock_guard<std::mutex> lg(mutex_);

        if (!dataQueue_.empty())
        {
            dataQueue_.pop_front();
            status = true;
        }

        return status;
    }

    /**
     *******************************************************************************
     * @brief clear - Empty data queue.
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

        dataQueue_.clear();
    }

    /**
     *******************************************************************************
     * @brief empty - Test whether the data queue is empty
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

        return dataQueue_.empty();
    }

    /**
     *******************************************************************************
     * @brief size - Get the size of the data queue
     *
     *  <1> Parameter Description:
     *
     *  @return Number of data objects in data queue
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    ElementSize_t size() const
    {
        std::lock_guard<std::mutex> lg(mutex_);

        return dataQueue_.size();
    }

    /**
     * No copy constructor.
     */
    ThreadSafeQueue(const ThreadSafeQueue&) = delete;

    /**
     * No assignment.
     */
    ThreadSafeQueue &operator=(const ThreadSafeQueue&) = delete;

private:
    std::deque<TYPE> dataQueue_;
    mutable std::mutex mutex_;
    std::condition_variable cond_;
};

}

#endif
