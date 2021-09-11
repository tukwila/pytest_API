/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Singleton.h
 * @brief  Definition of singleton mode
 *******************************************************************************
 */


#include <mutex>
#include <functional>   // std::bind
#include <utility>      // std::forward

#ifndef SINGLETON_H
#define SINGLETON_H

namespace roadDBCore
{


/**
 *******************************************************************************
 * @class Singleton
 *
 * @brief Implement singlton mode for wrapper class.
 *******************************************************************************
 */
template<typename INSTANCE_TYPE>
class Singleton
{
public:
    template<typename... Args>
    static INSTANCE_TYPE &getInstance(Args&& ... args)
    {
        std::call_once(s_buildFlag_, 
            std::bind(&buildInstance<Args&...>, std::forward<Args>(args)...)
        );

        return *s_pInstance_;
    }

    static void releaseInstance()
    {
        std::call_once(s_releaseFlag_, &freeInstance);
    }

    /**
     * No constructor.
     */
    Singleton() = delete;

    /**
     * No copy constructor.
     */
    Singleton(const Singleton&) = delete;

    /**
     * No assignment.
     */
    Singleton &operator=(const Singleton&) = delete;

private:
    template<typename... Args>
    static void buildInstance(Args&&... args)
    {
        if (!s_pInstance_)
        {
           s_pInstance_ = new INSTANCE_TYPE(std::forward<Args>(args)...);
        }
    }

    static void freeInstance()
    {
        if (s_pInstance_)
        {
            delete s_pInstance_;
            s_pInstance_ = nullptr;
        }
    }

private:
    static INSTANCE_TYPE *s_pInstance_;
    static std::once_flag s_buildFlag_;
    static std::once_flag s_releaseFlag_;
};

template <typename INSTANCE_TYPE>
INSTANCE_TYPE *Singleton<INSTANCE_TYPE>::s_pInstance_ = nullptr;

template <typename INSTANCE_TYPE>
std::once_flag Singleton<INSTANCE_TYPE>::s_buildFlag_;

template <typename INSTANCE_TYPE>
std::once_flag Singleton<INSTANCE_TYPE>::s_releaseFlag_;

}

#endif
