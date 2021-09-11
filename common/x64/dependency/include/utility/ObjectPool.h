/**
 *******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ObjectPool.h
 * @brief  Implementation of Object Pool
 *******************************************************************************
 */

#include <vector>
#include <functional> //std::bind
#include <iostream>
#include <type_traits>
#include <cassert>
#include "ThreadUtils/SpinLock.h"
#include "LogWrapper/LogWrapper.h"

#ifndef OBJECT_POOL_H
#define OBJECT_POOL_H

namespace roadDBCore
{

/**
 *******************************************************************************
 * @class BasePool
 *
 * @brief Provide interface for all derived object pools
 *******************************************************************************
 */

class BasePool
{
public:
    BasePool () = default;
    virtual ~BasePool() {}
    virtual void dump() = 0;
};

/**
 *******************************************************************************
 * @class ObjectPool
 *
 * @brief The object pool is implemented to manage all objects memory allocated
 *        when creating the pool. It provides interface to get shared pointer to
 *        a specified object from the object pool.
 *******************************************************************************
 */
template<typename OBJ_TYPE,
         typename ALLOC = std::allocator<OBJ_TYPE>,
         /*
             The SIZE_TYPE must be unsigned integral type,
             and the size of SIZE_TYPE must be no more than OBJ_TYPE,
             the total objects allocated must be less than the number
             taht the SIZE_TYPE can supported.
         */
         typename SIZE_TYPE = uint32_t>
class ObjectPool : public BasePool
{
public:
    ObjectPool(uint32_t num,
                    const std::string &name = "",
                    const ALLOC &alloc = ALLOC()):
                    BasePool(),
                    name_(name),
                    totalNum_(num),
                    alloc_(alloc)
    {
        static_assert(std::is_unsigned<SIZE_TYPE>::value, "The SIZE_TYPE must be unsigned integral type!");
        static_assert(sizeof(OBJ_TYPE) >= sizeof(SIZE_TYPE), "The size of OBJ_TYPE is less than SIZE_TYPE!");
        assert(num <= uint32_t(SIZE_TYPE(-1)));
        assert(num <= (std::size_t(-1) / sizeof(OBJ_TYPE)));
    }

    ~ObjectPool()
    {
        if (pObjBuffer_)
        {
            release();
        }
    }

    /**
     *******************************************************************************
     * @brief init - Init object pool: allocate memory for configured number of
     *               objects and link all free object together.
     *
     *  <1> Parameter Description:
     *
     *  @return true for success or return false.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool init()
    {
        try
        {
            pObjBuffer_ = alloc_.allocate(totalNum_);

            if (!pObjBuffer_)
            {
                COM_LOG_ERROR << name_ << "--Failed to allocate memory for objects pool!";
                return false;
            }
        }

        catch (std::exception &e)
        {
            COM_LOG_ERROR << name_ << "--Exception when allocating " << totalNum_ << " objects!";
            COM_LOG_ERROR << e.what();
            return false;
        }

        for (availIdx_ = 0; availIdx_ < totalNum_; ++availIdx_)
        {
            *(reinterpret_cast<SIZE_TYPE *>(&pObjBuffer_[availIdx_])) = availIdx_ + 1;

            //COM_LOG_TRACE << "&pObjBuffer_[" << availIdx_ << "]: " << &pObjBuffer_[availIdx_] << ", *pObjBuffer_[" << availIdx_ << "]: " << *(reinterpret_cast<SIZE_TYPE *>(&pObjBuffer_[availIdx_]));
        }

        availIdx_ = 0;
        availNum_ = totalNum_;
        COM_LOG_INFO << name_ << "--Allocate objects number: " << totalNum_;

        return true;
    }


    /**
     *******************************************************************************
     * @brief dump - Dump object pool info
     *
     *  <1> Parameter Description:
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void dump()
    {
        COM_LOG_INFO << name_
                     << ": totalNum " << totalNum_
                     << ", availNum " << availNum_
                     << ", availIdx " << availIdx_
                     << ", extraNum " << extraNum_;
    }

    /**
     *******************************************************************************
     * @brief getObj - Get shared pointer to the object of specified type
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - args   Template parameter pack.
     *
     *  @return shared pointer to the object
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    template<typename... Args>
    std::shared_ptr<OBJ_TYPE> getObj(Args&&... args) //return a share pointer
    {
        lock_.lock();

        if (availNum_)
        {
            OBJ_TYPE *pObject = &pObjBuffer_[availIdx_];

            //COM_LOG_TRACE << name_ << "--GetObj " << pObject << ", availIdx " << availIdx_ << " -> " << *reinterpret_cast<SIZE_TYPE *>(pObject) << ", availNum " << availNum_ - 1;

            --availNum_;
            availIdx_ = *reinterpret_cast<SIZE_TYPE *>(pObject);
            lock_.free();
            alloc_.construct(pObject, std::forward<Args>(args)...);

            return std::shared_ptr<OBJ_TYPE>(pObject,
                               std::bind(&ObjectPool<OBJ_TYPE, ALLOC, SIZE_TYPE>::releaseObj,
                                   this,
                                   std::placeholders::_1));
        }
        else
        {
            ++extraNum_;
            lock_.free();
            COM_LOG_WARN << name_ << "--No available object in the pool, extraNum " << extraNum_;

            return std::allocate_shared<OBJ_TYPE>(alloc_);
        }
    }

private:
    void releaseObj(OBJ_TYPE *pObj)
    {
        // In case of calling after destruction of the object pool
        if (!pObjBuffer_)
        {
            return;
        }

        SIZE_TYPE &nextIdx = *reinterpret_cast<SIZE_TYPE *>(pObj);
        SIZE_TYPE curIdx = static_cast<SIZE_TYPE>(pObj - pObjBuffer_);

        alloc_.destroy(pObj);

        lock_.lock();
        nextIdx = availIdx_;
        availIdx_ = curIdx;
        ++availNum_;
        lock_.free();
        //COM_LOG_TRACE << name_ << "--RelObj " << pObj << ", size " << sizeof(OBJ_TYPE) << ", availIdx " << curIdx << ", availNum " << availNum_;
    }

    void release()
    {
        dump();

        if (totalNum_ != availNum_)
        {
            std::vector<bool> vecFlag(totalNum_, false);
            SIZE_TYPE index = 0;
            SIZE_TYPE count = 0;

            for (; index < availNum_; ++index)
            {
                vecFlag[availIdx_] = true;
                availIdx_ = *reinterpret_cast<SIZE_TYPE *>(&pObjBuffer_[availIdx_]);
            }

            index = 0;

            for (; index < totalNum_; ++index)
            {
                if (!vecFlag[index])
                {
                    alloc_.destroy(&pObjBuffer_[index]);
                    ++count;
                    //COM_LOG_TRACE << name_ << "--Destroy object " << index;;
                }
            }

            COM_LOG_INFO << name_ << "--Destroy objects number " << count;
        }

        alloc_.deallocate(pObjBuffer_, totalNum_);
        availNum_ = 0;
        pObjBuffer_ = nullptr;
        COM_LOG_INFO << name_ << "--Deallocate objects number " << totalNum_;
    }

    /**
     * No copy constructor.
     */
    ObjectPool(const ObjectPool&) = delete;
    ObjectPool(const ObjectPool&&) = delete;

    /**
     * No assignment.
     */
    ObjectPool &operator=(const ObjectPool&) = delete;
    ObjectPool &operator=(const ObjectPool&&) = delete;

private:
    const std::string name_;
    const SIZE_TYPE totalNum_;
    SIZE_TYPE availNum_ = 0;
    SIZE_TYPE extraNum_ = 0;
    SIZE_TYPE availIdx_ = 0;
    OBJ_TYPE *pObjBuffer_ = nullptr;
    ALLOC alloc_;
    SpinLock lock_;
};



}

#endif
