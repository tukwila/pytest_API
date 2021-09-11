/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RWLock.h
 * @brief  RWLock 
 *******************************************************************************
 */
#pragma once

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <atomic>
namespace RDBVehicleAPI
{
#define LOCK_SUCCESS 0
#define LOCK_FAIL -1

enum RWLOCKOP_E
{
    RWLOCKOP_R_LOCK_E = 1,
    RWLOCKOP_R_UNLOCK_E = 2,
    RWLOCKOP_W_LOCK_E = 3,
    RWLOCKOP_W_UNLOCK_E = 4
};
class RWLock
{
public:
	RWLock();
	~RWLock();
   void lock(const RWLOCKOP_E op);
   bool init();
private:
	//Results are undefined if pthread_rwlock_init() is called specifying an already initialised read-write lock. 
	RWLock(RWLock const&);                          // Noncopyable
	RWLock& operator= (RWLock const&);    // Noncopyable
	void readUnlock();
	void writeUnlock();
	void readLock();
	void unlock();
	void writeLock();
private:
	pthread_rwlock_t mutex_;
	std::atomic<int>  intRlt_;
};

}