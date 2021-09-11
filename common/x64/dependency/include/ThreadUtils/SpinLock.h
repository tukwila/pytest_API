/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   SpinLock.h
 * @brief  Implementation of spin lock
 *******************************************************************************
 */

#include <atomic>

#ifndef SPIN_LOCK_H
#define SPIN_LOCK_H

namespace roadDBCore
{

/**
 *******************************************************************************
 * @class SpinLock
 *
 * @brief Implement spin lock.
 *******************************************************************************
 */
class SpinLock
{
public:
    SpinLock(): flag(ATOMIC_FLAG_INIT) {}

    /**
     *******************************************************************************
     * @brief lock - Get spin lock
     *
     *  <1> Parameter Description:
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void lock()
    {
        while (flag.test_and_set(std::memory_order_acquire));
    }

    /**
     *******************************************************************************
     * @brief free - Free spin lock
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
        flag.clear(std::memory_order_release);
    }

    /**
     * No copy constructor.
     */
    SpinLock(const SpinLock&) = delete;
    SpinLock(const SpinLock&&) = delete;

    /**
     * No assignment.
     */
    SpinLock &operator=(const SpinLock&) = delete;
    SpinLock &operator=(const SpinLock&&) = delete;

private:
    std::atomic_flag flag;
};


}

#endif
