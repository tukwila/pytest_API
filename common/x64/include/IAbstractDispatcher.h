/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IAbstractDispatcher.h
 * @brief  The class is the base class Dispatcher. The dispatcher dispatches
 *         a message to a work thread.
 *
 * Rivision History:
 *      Date              Submitter         Description
 *      2018.02.05        Tony Xiong		Initialization
 *******************************************************************************
 */
#ifndef COM_YGOMI_ROADDB_UTIL_IABSTRACTDISPATCHER_H_
#define COM_YGOMI_ROADDB_UTIL_IABSTRACTDISPATCHER_H_

#include <memory>

#include "Message.h"
#include "TSQueue.hpp"
#include "WorkThread.h"
#include "ThreadPool.h"

namespace RDBVehicleAPI
{

class WorkThread;

class IAbstractDispatcher {
public:
	/**
	 * Default Constructor
	 */
	IAbstractDispatcher();

	/**
	 * Constructor
	 *
	 * @param msgQueue The message queue to which this dispatcher is bound.
	 *
	 * Notes:
	 * When bound to msgQueue, this dispatcher will fetch message from msgQueue.
	 * If msgQueue is null, it is same as IAbstractDispatcher().
	 */
	IAbstractDispatcher(TSQueue<Message>* msgQueue);

	/**
	 * Constructor
	 *
	 * @param threadPool The thread pool to which this dispatcher is bound.
	 *
	 * Notes:
	 * When bound to threadPool, this dispatcher will fetch work thread from
	 * threadPool.
	 * If threadPool is null, it is same as IAbstractDispatcher().
	 */
	IAbstractDispatcher(ThreadPool* threadPool);

	/**
	 * Constructor
	 *
	 * @param msgQueue The message queue to which this dispatcher is bound.
	 * @param threadPool The thread pool to which this dispatcher is bound.
	 *
	 * Notes:
	 * 1. If msgQueue is null and threadPool is not, it is same as
	 * IAbstractDispatcher(ThreadPool* threadPool).
	 * 2. If threadPool is null and msgQueue is not, it is same as
	 * IAbstractDispatcher(TSQueue<Message>* msgQueue).
	 * 3. If both msgQueue and threadPool is null, it is same as
	 * IAbstractDispatcher().
	 */
	IAbstractDispatcher(TSQueue<Message>* msgQueue, ThreadPool* threadPool);

	/**
	 * Desctructor
	 */
	virtual ~IAbstractDispatcher();

	/**
	 * Get the work object which is supposed to handle the given message.
	 *
	 * @param message The specified message.
	 * @return The work object which is supposed to handle the given message.
	 *
	 * Notes:
	 * 1. Normally this method shall return a specific type of Work object
	 *    according to a specific type of message.
	 * 2. This is an abstract method. Implement this method to customize
	 *    message-to-work mapping.
	 */
	virtual std::shared_ptr<Work> getWork(std::shared_ptr<Message> message) = 0;

	/**
	 * Fetech a message from the message queue (given in the constructor)
	 * and then dispatch it to a work thread fetched from the work pool
	 * (given in the constructor).
	 * This method shall be used when the instance is created
	 * by IAbstractDispatcher(TSQueue<Message>* msgQueue, ThreadPool* threadPool).
	 * If not so, nothing will be done.
	 *
	 * @return If either no message is fetched or no idle work thread is fetched,
	 *         return false, otherwise return true.
	 *
	 * Notes:
	 * What this method does is actually:
	 * 1. Fetch a message from the message queue (given in the constructor);
	 * 2. Fetch a work thread from the thread pool (given in the constructor);
	 * 3. Call getWork() with the fetched message to get the corresponding work;
	 * 4. Call setInputMsg() on the work object to attach the message to the work;
	 * 5. Call doWork() on the fetched work thread with the work given as argument
	 *    to fulfill the work.
	 */
	virtual bool dispatch();

	/**
	 * Dispatch the given message to a work thread fetched from
	 * the thread pool (given in the constructor).
	 * This method shall be used when a thread pool is given when this
	 * instance is created. If not so, nothing will be done.
	 *
	 * @param message The message to be dispatched.
	 * @return If message is null or no idle work thread is fetched,
	 *         return false, otherwise return true.
	 *
	 * Notes:
	 * What this method does is actually:
	 * 1. Fetch a work thread from the thread pool (given in the constructor);
	 * 2. Call getWork() with the given message to get the corresponding work;
	 * 3. Call setInputMsg() on the work object to attach the message to the work;
	 * 4. Call doWork() on the fetched work thread with the work given as argument
	 *    to fulfill the work.
	 */
	virtual bool dispatch(std::shared_ptr<Message> message);

	/**
	 * Fetch a message from the message pool  (given in the constructor)
	 * and dispatch it to the given work thread.
	 * This method shall be used when a message queue is given when this
	 * instance is created. If not so, nothing will be done.
	 *
	 * @param workThread The work thread to be dispatched to.
	 * @return If workThread is null or no message is fetched,
	 *         return false, otherwise return true.
	 *
	 * Notes:
	 * What this method does is actually:
	 * 1. Fetch a message from the message pool (given in the constructor);
	 * 2. Call getWork() with the fetched message to get the corresponding work;
	 * 3. Call setInputMsg() on the work object to attach the message to the work;
	 * 4. Call doWork() on the given work thread with the work given as argument
	 *    to fulfill the work.
	 */
	virtual bool dispatch(WorkThread* workThread);

	/**
	 * Dispatch the given message to the given work thread.
	 * This method shall be used when the instance is created
	 * by IAbstractDispatcher(). If not so, nothing will be done.
	 *
	 * @param message The message to be dispatched.
	 * @param workThread The message to be dispatched to.
	 * @return If message is null or workThread is null,
	 *         return false, otherwise return true.
	 *
	 * Notes:
	 * What this method does is actually:
	 * 1. Call getWork() to get corresponding work of the message;
	 * 2. Call setInputMsg() on the work object to attach the message to the work;
	 * 3. Call doWork() on the given work thread with the work given as argument
	 *    to fulfill the work.
	 */
	virtual bool dispatch(std::shared_ptr<Message> message, WorkThread* workThread);

private:
	TSQueue<Message>* _msgQueue;
	ThreadPool* _threadPool;
};

// } /* namespace util */
} /* namespace roaddb */


#endif /* COM_YGOMI_ROADDB_UTIL_IABSTRACTDISPATCHER_H_ */
