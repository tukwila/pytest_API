/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   SingletonManager.h
 * @brief  Implementation of singleton manager
 *******************************************************************************
 */

#include <map>
#include <memory>
#include <sstream>
#include "utility/Singleton.h"
#include "LogWrapper/LogWrapper.h"

#ifndef SINGLETON_MANAGER_H
#define SINGLETON_MANAGER_H

namespace roadDBCore
{

/**
 *******************************************************************************
 * @class BaseWrapper
 *
 * @brief Base wrapper class for singleton object
 *******************************************************************************
 */
class BaseWrapper
{
public:
    BaseWrapper(void *pObjectIn): pObject_(pObjectIn)
    {
        COM_LOG_TRACE << "Object address: " << pObject_;
    };

    virtual ~BaseWrapper()
    {
        pObject_ = nullptr;
    };

    void *get() {return pObject_;}

private:
    void *pObject_;
};

/**
 *******************************************************************************
 * @class ObjectWrapper
 *
 * @brief Wrapper class for concrete singleton object
 *******************************************************************************
 */
template<typename T>
class ObjectWrapper: public BaseWrapper
{
public:
    template<typename... Args>
    ObjectWrapper(Args&&... args):
        BaseWrapper(static_cast<void *>(new T(std::forward<Args>(args)...)))
    {
        COM_LOG_TRACE << "Create object: " << typeid(T).name();
    }

    virtual ~ObjectWrapper()
    {
        T* pObject = static_cast<T*>(get());

        if (pObject)
        {
            delete pObject;
            COM_LOG_TRACE << "Delete object: " << typeid(T).name();
        }
    }
};

/**
 *******************************************************************************
 * @class SingletonManager
 *
 * @brief Implement singlton manager to manage all singleton objects
 *******************************************************************************
 */
class SingletonManager
{
public:
    SingletonManager() = default;

    ~SingletonManager() {free();}

    /**
     *******************************************************************************
     * @brief get - Get the singleton object of specified type
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - args   Template parameter pack.
     *
     *  @return Number of singleton objects
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    template<typename T, typename... Args>
    T &get(Args&&... args)
    {
        auto const &name = typeid(T).name();
        BaseWrapper* pBase = mapObjects_[name];

        if (!pBase)
        {
            ObjectWrapper<T> *pWrapper =
                    &Singleton<ObjectWrapper<T>>::getInstance(std::forward<Args>(args)...);

            pBase = static_cast<BaseWrapper*>(pWrapper);
            mapObjects_[name] = pBase;
            dump();
        }

        return *static_cast<T*>(pBase->get());
    }

    /**
     *******************************************************************************
     * @brief free - Free all singleton objects
     *
     *  <1> Parameter Description:
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void free()
    {
        for (auto &pairItem : mapObjects_)
        {
            if (pairItem.second)
            {
                delete pairItem.second;
            }
        }

        mapObjects_.clear();
    }

    /**
     *******************************************************************************
     * @brief size - Get total number of singleton objects
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
        return mapObjects_.size();
    }

    /**
     *******************************************************************************
     * @brief dump - Dump detail info of all singleton objects
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
        std::stringstream ss;

        ss << size() << " singleton objects have been created: \n";

        for (auto const &pairItem : mapObjects_)
        {
            ss << pairItem.first << " object address: " << static_cast<void *>(pairItem.second) << "\n";
        }

        COM_LOG_DEBUG << ss.str();
    }

    /**
     * No copy constructor.
     */
    SingletonManager(const SingletonManager&) = delete;
    SingletonManager(const SingletonManager&&) = delete;

    /**
     * No assignment.
     */
    SingletonManager &operator=(const SingletonManager&) = delete;
    SingletonManager &operator=(const SingletonManager&&) = delete;

private:
    std::map<std::string, BaseWrapper*> mapObjects_;
};

using SingleMgr_t = Singleton<SingletonManager>;
extern std::shared_ptr<roadDBCore::SingletonManager> g_hSingleMgr;


}

#endif
