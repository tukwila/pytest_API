/**
 *******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ObjectManager.h
 * @brief  Implementation of singleton manager
 *******************************************************************************
 */

#include <map>
#include <memory>
#include <sstream>
#include <type_traits>
#include <cassert>
#include "utility/ObjectPool.h"
#include "LogWrapper/LogWrapper.h"

#ifndef OBJECT_MANAGER_H
#define OBJECT_MANAGER_H

namespace roadDBCore
{

/**
 *******************************************************************************
 * @class ObjectManager
 *
 * @brief Implement object manager to manage all object pools and get object
 *        from specified pool.
 *******************************************************************************
 */
class ObjectManager
{
public:
    ObjectManager() = default;

    ~ObjectManager() = default;

    /**
     *******************************************************************************
     * @brief createPool - Create object pool and cache the handle of the pool.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - num   Total objects allocated in the object pool.
     *
     *  @param [In]  - alloc Allocator which is used to control the allocation
     *                       of objects in the object pool.
     *
     *  @return true for success or return false.
     *
     *  <2> Detailed Description:
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
    bool createPool(std::size_t num, const ALLOC &alloc = ALLOC())
    {
        static_assert(std::is_unsigned<SIZE_TYPE>::value, "The SIZE_TYPE must be unsigned integral type!");

        auto const &name = typeid(OBJ_TYPE).name();

        if (mapObjPools_.find(name) != mapObjPools_.end())
        {
            COM_LOG_ERROR << "The object pool has been created for type: " << name;

            return false;
        }

        std::shared_ptr<ObjectPool<OBJ_TYPE, ALLOC, SIZE_TYPE>> spPool =
                    std::make_shared<ObjectPool<OBJ_TYPE, ALLOC, SIZE_TYPE>>(num, name);

        if (!spPool || !spPool->init())
        {
            COM_LOG_ERROR << "Failed to create the object pool: " << name;

            return false;
        }

        std::shared_ptr<BasePool> spBasePool = std::static_pointer_cast<BasePool>(spPool);

        mapObjPools_[name] = spBasePool;
        dump();

        return true;
    }

    /**
     *******************************************************************************
     * @brief getObj - Get handle to object pool of specified type
     *
     *  <1> Parameter Description:
     *
     *  @return shared pointer to the object
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    template<typename OBJ_TYPE,
             typename ALLOC = std::allocator<OBJ_TYPE>,
             /*
                 The SIZE_TYPE must be unsigned data type,
                 and the size of SIZE_TYPE must be no more than OBJ_TYPE,
                 the total objects allocated must be less than the number
                 taht the SIZE_TYPE can supported.
             */
             typename SIZE_TYPE = uint32_t>
    std::shared_ptr<ObjectPool<OBJ_TYPE, ALLOC, SIZE_TYPE>> getPool()
    {
        auto const &name = typeid(OBJ_TYPE).name();
        auto &spBasePool = mapObjPools_[name];

        if (!spBasePool)
        {
            COM_LOG_ERROR << "No object pool has been created for type: " << name;
            dump();

            return nullptr;
        }

        return std::static_pointer_cast<ObjectPool<OBJ_TYPE, ALLOC, SIZE_TYPE>>(spBasePool);
    }

    /**
     *******************************************************************************
     * @brief get - Get shared pointer to object
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - args   Template parameter pack used to construct the object.
     *
     *  @return shared pointer to object
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    template<typename OBJ_TYPE,
             typename ALLOC = std::allocator<OBJ_TYPE>,
             /*
                 The SIZE_TYPE must be unsigned data type,
                 and the size of SIZE_TYPE must be no more than OBJ_TYPE,
                 the total objects allocated must be less than the number
                 taht the SIZE_TYPE can supported.
             */
             typename SIZE_TYPE = uint32_t,
             typename... Args>
    std::shared_ptr<OBJ_TYPE> getObj(Args&&... args)
    {
        auto spPool = getPool<OBJ_TYPE, ALLOC, SIZE_TYPE>();

        if (spPool)
        {
            return spPool->getObj(std::forward<Args>(args)...);
        }
        else
        {
            return nullptr;
        }
    }

    /**
     *******************************************************************************
     * @brief get - Get specified number of objects
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut]  - vecSpObjs   Objects.
     *
     *  @param [in]  - num   The number of objects requested.
     *
     *  @param [In]  - args   Template parameter pack used to construct the object.
     *
     *  @return The number of objects returned.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    template<typename OBJ_TYPE,
             typename ALLOC = std::allocator<OBJ_TYPE>,
             /*
                 The SIZE_TYPE must be unsigned data type,
                 and the size of SIZE_TYPE must be no more than OBJ_TYPE,
                 the total objects allocated must be less than the number
                 taht the SIZE_TYPE can supported.
             */
             typename SIZE_TYPE = uint32_t,
             typename... Args>
    uint32_t getObj(std::vector<std::shared_ptr<OBJ_TYPE>> vecSpObjs, uint32_t num, Args&&... args)
    {
        uint32_t retNum = 0;
        auto spPool = getPool<OBJ_TYPE, ALLOC, SIZE_TYPE>();

        if (spPool)
        {
            for (; retNum < num; )
            {
                auto spObj = spPool->getObj(std::forward<Args>(args)...);

                if (spObj)
                {
                    vecSpObjs.push_back(spObj);
                    ++retNum;
                }
                else
                {
                    break;
                }
            }
        }

        return retNum;
    }

    /**
     *******************************************************************************
     * @brief size - Get total number of object pools
     *
     *  <1> Parameter Description:
     *
     *  @return Number of singleton objects
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    std::size_t size() const
    {
        return mapObjPools_.size();
    }

    /**
     *******************************************************************************
     * @brief dump - Dump detail info of all object pools
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
        COM_LOG_INFO << size() << " object pools have been created:";

        for (auto const &pairItem : mapObjPools_)
        {
            if (pairItem.second)
            {
                pairItem.second->dump();
            }
        }
    }

    /**
     * No copy constructor.
     */
    ObjectManager(const ObjectManager&) = delete;
    ObjectManager(const ObjectManager&&) = delete;

    /**
     * No assignment.
     */
    ObjectManager &operator=(const ObjectManager&) = delete;
    ObjectManager &operator=(const ObjectManager&&) = delete;

private:
    std::map<std::string, std::shared_ptr<BasePool>> mapObjPools_;
};


}

#endif
