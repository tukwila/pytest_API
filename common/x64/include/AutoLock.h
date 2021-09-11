/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   AutoLock.h
 * @brief  AutoLock
 *******************************************************************************
 */

#pragma once

#include <pthread.h>

namespace RDBVehicleAPI
{
class Mutex 
{
public:
    Mutex();
    ~Mutex();
private:
    friend class Lock;
    pthread_mutex_t mutex_;
};

class Lock 
{
public:
    Lock(Mutex& m);
    ~Lock();
private:
    Mutex& mutexRef_;
};
// }
}
