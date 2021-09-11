/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   WorkThread.h
 * @brief  This class is used to let a Work instance run in a thread.
 *
 * Rivision History:
 *      Date              Submitter         Description
 *      2018.02.05        Tony Xiong		Initialization
 *******************************************************************************
 */
#ifndef COM_YGOMI_ROADDB_UTIL_WORKTHREAD_H_
#define COM_YGOMI_ROADDB_UTIL_WORKTHREAD_H_
#include <thread>
#include <mutex>
#include <condition_variable>

#include "Work.h"
#include "ThreadPool.h"

namespace RDBVehicleAPI
{

class WorkThread {
friend class ThreadPool;

public:
	/**
	 * When the thread is initially created (Value: 1).
	 */
	static const int STATUS_INIT;

	/**
	 * When this work thread is ready to do work  (Value: 2).
	 * Note the work thread in this status is suspended.
	 * It can be resumed by doWork().
	 */
	static const int STATUS_IDLE;

	/**
	 * When this work thread is doing its work (Value: 3).
	 * Note after the work is done, this work thread will
	 * be suspended again, and its status will change back
	 * to STATUS_IDLE again.
	 */
	static const int STATUS_WORKING;

	/**
	 * When the thread terminates its working loop, i.e.: it
	 * will be terminated and won't do work no more. (Value: 4)
	 */
	static const int STATUS_TERMINATED;

	/**
	 * Destructor
	 */
	virtual ~WorkThread();

	/**
	 * Get the status of the work thread.
	 *
	 * @return The status of the work thread.
	 *
	 * Refer to the statuses defined in this class.
	 */
	virtual int getStatus() const {return _status;};

	/**
	 * Start to do some work.
	 *
	 * @param[IN] work The work to do.
	 *
	 * Notes:
	 * 1. This method will resume this work thread (while it is suspended)
	 *    and let it do the given work.
	 * 2. This method only takes effect when the status of this work thread
	 *    is STATUS_IDLE, or nothing will happen.
	 * 3. After this work thread is resumed by this method, the status will
	 *    become STATUS_WORKING; and once the work is done, the status will
	 *    change back to STATUS_IDLE and this work thread will be suspended
	 *    again.
	 */
	virtual void doWork(std::shared_ptr<Work> work);

private:

	/**
	 * Constructor
	 */
	WorkThread();

	/**
	 * Constructor
	 *
	 * @param id The id of the work thread.
	 * @param threadPool The thread pool under which the work thread
	 *                   is managed. If this param is given, the
	 *                   work thread will be reclaimed (as idle thread)
	 *                   at the end of doWork().
	 */
	explicit WorkThread(uint32_t id, ThreadPool* threadPool);

	WorkThread(const WorkThread& workThread);
	WorkThread& operator=(const WorkThread& workThread);

	/**
	 * Get the id of the work thread.
	 *
	 * @return The id of the work thread.
	 */
	int getId() const {return _id;};

	/**
	 * Get the thread pool under which this work thread is managed.
	 *
	 * @return The pointer to the thread pool, or null if the work thread
	 *         is not under the control of a thread pool.
	 */
	ThreadPool* getPool() const {return pThreadPool_;};

	/**
	 * Start the work thread.
	 *
	 * Notes:
	 * 1. A thread will be created first by this method, and
	 *    then it will be suspended until doWork().
	 * 2. Note at the end of this method, the work thread
	 *    status will be set to STATUS_IDLE.
	 */
	void start();

	/**
	 * Stop the work thread.
	 *
	 * Notes:
	 * 1. The thread created in start() will be terminated by
	 *    this method.
	 *    This method will wait the termination of the thread.
	 * 2. This method takes no effect when the work thread
	 *    is already in STATUS_TERMINATED.
	 */
	void stop();

	bool shouldStop() { return _exit == true; }
	void setExit(bool v) { _exit = v; } 

//	/**
//	 * Same as std::thread::join().
//	 *
//	 * Note this method only takes effect when the work thread is created using
//	 * WorkThread().
//	 */
//	virtual void join();
//
//	/**
//	 * Same as std::thread::detach().
//	 *
//	 * Note this method only takes effect when the work thread is created using
//	 * WorkThread().
//	 */
//	virtual void detach();

	virtual void exec();

	uint32_t _id;
	ThreadPool* pThreadPool_;
    int _status = STATUS_INIT;
	bool _exit = false;
	std::thread* _thread;
	std::shared_ptr<Work> _work;
	std::mutex _mutex;
	std::condition_variable _cv;
	bool _fetched = false;
};

// } /* namespace util */
} /* namespace roaddb */


#endif /* COM_YGOMI_ROADDB_UTIL_WORKTHREAD_H_ */
