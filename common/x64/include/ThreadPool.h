/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ThreadPool.h
 * @brief  This class manages a pool of work threads.
 *
 * Rivision History:
 *      Date              Submitter         Description
 *      2018.02.05        Tony Xiong		Initialization
 *******************************************************************************
 */
#ifndef COM_YGOMI_ROADDB_UTIL_THREADPOOL_H_
#define COM_YGOMI_ROADDB_UTIL_THREADPOOL_H_
#include <stdint.h>
#include <mutex>
#include <queue>
#include <map>
//#include <memory>

namespace RDBVehicleAPI
{

class WorkThread;

class ThreadPool {
friend class WorkThread;

public:
	/**
	 * Constructor
	 *
	 * @param maxSize The size of the thread pool.
	 * Default value is 8.
	 */
	explicit ThreadPool(uint32_t size = 8);

	/**
	 * Destructor
	 */
	virtual ~ThreadPool();

	/**
	 * The size of the thread pool.
	 *
	 * @return The size of the thread pool.
	 */
	virtual uint32_t getSize();

	/**
	 * The count of idle threads.
	 *
	 * @return The count of idle threads.
	 */
	virtual uint32_t getIdleCount();

	/**
	 * Fetch an idle thread from the thread pool.
	 *
	 * @return The pointer to the idle thread,
	 *         or null if no idle thread available.
	 */
	virtual WorkThread* fetch();
//	std::shared_ptr<WorkThread> getThread(uint32_t id);

private:
	ThreadPool(const ThreadPool& threadPool);
	ThreadPool& operator=(const ThreadPool& threadPool);

	/**
	 * Reclaim a thread as idle.
	 *
	 * @param The thread to be reclaimed.
	 * @return True if success, false otherwise.
	 *
	 * Success when all below conditions are satisified:
	 * The given thread is managed by this thread pool;
	 * The given thread is fetched by this thread pool;
	 * The status of the given thread is STATUS_IDLE.
	 *
	 * Note after being reclaimed, the given thread will
	 * be flagged as relaimed (i.e.: not fetched). So relaim
	 * on a relaimed thread will do nothing.
	 */
	virtual bool reclaim(WorkThread* thread);


	uint32_t _size;
	std::mutex _mutex;
	std::queue<WorkThread*> _idleQueue;
	std::map<uint32_t, WorkThread*> _threadMap;

};

} 


#endif /* COM_YGOMI_ROADDB_UTIL_THREADPOOL_H_ */
