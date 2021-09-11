/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018-2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   LoopQueue.h
 * @brief  Implementation of lock free loop queue and thread safe loop queue.
 *******************************************************************************
 */


#include <atomic>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include "LogWrapper/LogWrapper.h"
#include "typeDef.h"
#include "ThreadUtils/EventManager.h"
#include "utility/SingletonManager.h"

#ifndef LOOP_QUEUE_H
#define LOOP_QUEUE_H


namespace roadDBCore
{

const uint32_t DEFAULT_BUFFER_SIZE = 300;


/**
 *******************************************************************************
 * @class LockFreeLoopQueue
 *
 * @brief Support sharing data objects between one writing thread and one reading
 *        thread.
 *  1. There is no lock for all operations.
 *  2. The queue is implemented by loop buffer.
 *******************************************************************************
 */
template<typename TYPE>
class LockFreeLoopQueue
{
public:
    using ElementSize_t = typename std::vector<TYPE>::size_type;

    /**
     *******************************************************************************
     * @brief constructor
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - vecCapacity   The reserved number of objects in loop queue.
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    explicit LockFreeLoopQueue(ElementSize_t vecCapacity = DEFAULT_BUFFER_SIZE):
                                       vecCapacity_(vecCapacity),
                                       atomAvailNumber_(0),
                                       bOverflow_(false),
                                       readIndex_(0),
                                       writeIndex_(0)
    {
        if (0 == vecCapacity)
        {
            LOG_WARN << "Invalid buffer size -- " << vecCapacity
                     << ", has been reconfig it to "
                     << DEFAULT_BUFFER_SIZE;

            vecCapacity_ = DEFAULT_BUFFER_SIZE;
        }

        vObjectBuffer_.resize(vecCapacity_);

        LOC_LOG_INFO << "Construct LockFreeLoopQueue, size: " << vecCapacity_;
    }

    virtual ~LockFreeLoopQueue() {}

    /**
     *******************************************************************************
     * @brief write - Save an object into the loop queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - object   Data object to be saved
     *
     *  @return True if success, or return false for full and send Overflow event.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    bool write(const TYPE &object)
    {
        bool status = true;
        ElementSize_t availNumber = atomAvailNumber_.load();

        if (availNumber == vecCapacity_)
        {
            COM_LOG_WARN << "Failed to put object, the buffer is full!";

            status = false;

            if (!bOverflow_.load())
            {
                bOverflow_.store(true);
                COM_LOG_WARN << "!!! Post Overflow event.";
                g_hSingleMgr->get<EventManager>().pushSyncEvent(std::make_shared<OverflowEvent_t>());
            }
        }
        else
        {
            vObjectBuffer_[writeIndex_] = object;
            ++writeIndex_;
            writeIndex_ %= vecCapacity_;
            ++atomAvailNumber_;
        }

        return status;
    }

    /**
     *******************************************************************************
     * @brief write - Save a vector of objects into the loop queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - vObjects   Data objects to be saved
     *
     *  @return The number of objects saved, or send Overflow event for overflow.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    ElementSize_t write(const std::vector<TYPE> &vObjects)
    {
        ElementSize_t availNumber = atomAvailNumber_.load();
        ElementSize_t freeNum = vecCapacity_ - availNumber;
        ElementSize_t objNum = vObjects.size();

        if (freeNum < objNum)
        {
            COM_LOG_WARN  << "Some objects would be discarded! "
                          << "The free buffer number ("
                          << freeNum
                          << ") is less than committed number ("
                          << objNum
                          << ")!";

            objNum = freeNum;

            if (!bOverflow_.load())
            {
                bOverflow_.store(true);
                COM_LOG_WARN << "!!! Post Overflow event.";
                g_hSingleMgr->get<EventManager>().pushSyncEvent(std::make_shared<OverflowEvent_t>());
            }
        }

        for (ElementSize_t i = 0; i < objNum; ++i)
        {
            vObjectBuffer_[(writeIndex_ + i) % vecCapacity_] = vObjects[i];
        }

        writeIndex_ = (writeIndex_ + objNum) % vecCapacity_;
        atomAvailNumber_.fetch_add(objNum);

        return objNum;
    }

    /**
     *******************************************************************************
     * @brief read - Read an object from the loop queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - object   Data object to be read
     *
     *  @return True if success, or return false if empty.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    bool read(TYPE &object)
    {
        ElementSize_t availNumber = atomAvailNumber_.load();

        if (0 == availNumber)
        {
            //COM_LOG_TRACE << "The buffer is empty!";

            return false;
        }
        else
        {
            object = vObjectBuffer_[readIndex_];
            ++readIndex_;
            readIndex_ %= vecCapacity_;
            --atomAvailNumber_;

            return true;
        }
    }

    /**
     *******************************************************************************
     * @brief read - Get all available objects in the loop queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - vObjects   Data objects to be read
     *
     *  @return The number of objects read from the queue.
     *
     *  <2> Detailed Description: It's not thread-safe.
     *      Can't be called by multi-threads at the same time.
     *******************************************************************************
     */
    ElementSize_t read(std::vector<TYPE> &vObjects)
    {
        ElementSize_t objNum = atomAvailNumber_.load();

        if (objNum)
        {
            vObjects.reserve(objNum);

            for (ElementSize_t i = 0; i < objNum; ++i)
            {
                vObjects.push_back(vObjectBuffer_[(readIndex_ + i) % vecCapacity_]);
            }

            readIndex_ = (readIndex_ + objNum) % vecCapacity_;
            atomAvailNumber_.fetch_sub(objNum);
        }

        return objNum;
    }

    /**
     *******************************************************************************
     * @brief peek - Just get the first data object from loop queue without removing.
     *  If the loop queue is empty, then return flase without waiting.
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

        if (atomAvailNumber_.load())
        {
            object = vObjectBuffer_[readIndex_];
            status = true;
        }

        return status;
    }

    /**
     *******************************************************************************
     * @brief pop - Remove the first data object from the loop queue
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

        if (atomAvailNumber_.load())
        {
            ++readIndex_;
            readIndex_ %= vecCapacity_;
            --atomAvailNumber_;
            status = true;
        }

        return status;
    }

    /**
     *******************************************************************************
     * @brief clear - Empty loop queue.
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
        readIndex_ = 0;
        writeIndex_ = 0;
        atomAvailNumber_.store(0);
        bOverflow_.store(false);
    }

    /**
     *******************************************************************************
     * @brief empty - Test whether the loop queue is empty
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
        return !atomAvailNumber_.load();
    }

    /**
     *******************************************************************************
     * @brief full - Test whether the loop queue is full
     *
     *  <1> Parameter Description:
     *
     *  @return True if full, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool full() const
    {
        return (atomAvailNumber_.load() == vecCapacity_);
    }

    /**
     *******************************************************************************
     * @brief size - Get the element size of the queue
     *
     *  <1> Parameter Description:
     *
     *  @return Number of elements in the queue
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    ElementSize_t size() const
    {
        return atomAvailNumber_.load();
    }

    /**
     * No copy constructor.
     */
    LockFreeLoopQueue(const LockFreeLoopQueue&) = delete;

    /**
     * No assignment.
     */
    LockFreeLoopQueue &operator=(const LockFreeLoopQueue&) = delete;

private:
    ElementSize_t vecCapacity_;
    std::atomic<ElementSize_t> atomAvailNumber_;
    std::atomic<bool> bOverflow_;
    ElementSize_t readIndex_;
    ElementSize_t writeIndex_;
    std::vector<TYPE> vObjectBuffer_;
};

/**
 *******************************************************************************
 * @class ThreadSafeLoopQueue
 *
 * @brief Support sharing data objects among multi writing threads and multi
 *        reading threads. The queue is implemented based on loop buffer.
 *******************************************************************************
 */
template<typename TYPE>
class ThreadSafeLoopQueue
{
public:
    using ElementSize_t = typename std::vector<TYPE>::size_type;
    using Mutex_t = std::mutex;//std::recursive_mutex;

    /**
     *******************************************************************************
     * @brief constructor
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - vecCapacity   The reserved number of objects in loop queue.
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    explicit ThreadSafeLoopQueue(ElementSize_t vecCapacity = DEFAULT_BUFFER_SIZE):
                                         bOverflow_(false),
                                         vecCapacity_(vecCapacity),
                                         availNumber_(0),
                                         readIndex_(0),
                                         writeIndex_(0)
    {
        if (0 == vecCapacity)
        {
            LOG_WARN << "Invalid buffer size -- " << vecCapacity
                     << ", has been reconfig it to "
                     << DEFAULT_BUFFER_SIZE;

            vecCapacity_ = DEFAULT_BUFFER_SIZE;
        }

        vObjectBuffer_.resize(vecCapacity_);

        LOC_LOG_INFO << "Construct ThreadSafeLoopQueue, size: " << vecCapacity_;
    }

    virtual ~ThreadSafeLoopQueue() {}

    /**
     *******************************************************************************
     * @brief push - Save the data object into loop queue.
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
    bool push(const TYPE &object)
    {
        std::lock_guard<Mutex_t> lg(mutex_);

        return unsafePush(object);
    }

    /**
     *******************************************************************************
     * @brief pull - Retrieve (get and remove) data object from loop queue.
     *  If the loop queue is empty, then waiting until any data object is ready.
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
        std::unique_lock<Mutex_t> ul(mutex_);

        cond_.wait(ul, [this] {return availNumber_;});
        object = vObjectBuffer_[readIndex_];
        ++readIndex_;
        readIndex_ %= vecCapacity_;
        --availNumber_;

        return true;
    }

    /**
     *******************************************************************************
     * @brief pull - Retrieve (get and remove) data object from loop queue
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
        std::unique_lock<Mutex_t> ul(mutex_);

        if (waitMilliSeconds)
        {
            std::chrono::milliseconds waitTime(waitMilliSeconds);

            cond_.wait_for(ul, waitTime, [this] {return availNumber_;});
        }

        if (availNumber_)
        {
            object = vObjectBuffer_[readIndex_];
            ++readIndex_;
            readIndex_ %= vecCapacity_;
            --availNumber_;
            status = true;
        }

        return status;
    }

    /**
     *******************************************************************************
     * @brief peek - Just get the first data object from loop queue without removing.
     *  If the loop queue is empty, then return flase without waiting.
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
        std::lock_guard<Mutex_t> lg(mutex_);

        if (availNumber_)
        {
            object = vObjectBuffer_[readIndex_];
            status = true;
        }

        return status;
    }

    /**
     *******************************************************************************
     * @brief peekLast - Just get the last data object from loop queue without removing.
     *  If the loop queue is empty, then return flase without waiting.
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
    bool peekLast(TYPE &object) const
    {
        std::lock_guard<Mutex_t> lg(mutex_);

        return unsafePeekLast(object);
    }

    /**
     *******************************************************************************
     * @brief pop - Remove the first data object from the loop queue
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
        std::lock_guard<Mutex_t> lg(mutex_);

        if (availNumber_)
        {
            unsafePop();
            status = true;
        }

        return status;
    }

    /**
     *******************************************************************************
     * @brief clear - Empty loop queue.
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
        std::lock_guard<Mutex_t> lg(mutex_);

        unsafeClear();
    }

    /**
     *******************************************************************************
     * @brief empty - Test whether the loop queue is empty
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
        std::lock_guard<Mutex_t> lg(mutex_);

        return unsafeEmpty();
    }

    /**
     *******************************************************************************
     * @brief full - Test whether the loop queue is full
     *
     *  <1> Parameter Description:
     *
     *  @return True if full, or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool full() const
    {
        std::lock_guard<Mutex_t> lg(mutex_);

        return (availNumber_ == vecCapacity_);
    }

    /**
     *******************************************************************************
     * @brief size - Get the element size of the queue
     *
     *  <1> Parameter Description:
     *
     *  @return Number of elements in the queue
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    ElementSize_t size() const
    {
        std::lock_guard<Mutex_t> lg(mutex_);

        return availNumber_;
    }

protected:
    void unsafeClear()
    {
        readIndex_ = 0;
        writeIndex_ = 0;
        availNumber_ = 0;
        bOverflow_ = false;
    }

    bool unsafeEmpty() const {return !availNumber_;}

    bool unsafePeekLast(TYPE &object) const
    {
        bool status = false;

        if (availNumber_)
        {
            object = vObjectBuffer_[(readIndex_ + availNumber_ - 1) % vecCapacity_];
            status = true;
        }

        return status;
    }

    bool unsafePush(const TYPE &object)
    {
        bool status = true;

        if (availNumber_ < vecCapacity_)
        {
            vObjectBuffer_[writeIndex_] = object;
            ++writeIndex_;
            writeIndex_ %= vecCapacity_;
            ++availNumber_;
            cond_.notify_all();
        }
        else
        {
            COM_LOG_WARN << "Failed to put object, the buffer is full!";
            status = false;

            if (!bOverflow_)
            {
                bOverflow_ = true;
                COM_LOG_WARN << "!!! Post Overflow event.";
                g_hSingleMgr->get<EventManager>().pushSyncEvent(std::make_shared<OverflowEvent_t>());
            }
        }

        return status;
    }

    void unsafePop()
    {
        ++readIndex_;
        readIndex_ %= vecCapacity_;
        --availNumber_;
    }

    /**
     * No copy constructor.
     */
    ThreadSafeLoopQueue(const ThreadSafeLoopQueue&) = delete;

    /**
     * No assignment.
     */
    ThreadSafeLoopQueue &operator=(const ThreadSafeLoopQueue&) = delete;

protected:
    bool bOverflow_;
    ElementSize_t vecCapacity_;
    ElementSize_t availNumber_;
    ElementSize_t readIndex_;
    ElementSize_t writeIndex_;
    std::vector<TYPE> vObjectBuffer_;
    mutable Mutex_t mutex_;
    std::condition_variable cond_;
};


}// namespace roadDBCore




#endif //LOOP_QUEUE_H


