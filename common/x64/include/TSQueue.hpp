/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   TSQueue.hpp
 * @brief  This class enhances the std::queue to make it thread-safe.
 *
 * Rivision History:
 *      Date              Submitter         Description
 *      2018.02.05        Tony Xiong		Initialization
 *      2020.03.06        Yajun Zhao		add clear function
 *******************************************************************************
 */

#ifndef COM_YGOMI_ROADDB_UTIL_TSQUEUE_HPP_
#define COM_YGOMI_ROADDB_UTIL_TSQUEUE_HPP_

#include <queue>
#include <mutex>
#include <functional>
#include <list>
#include <memory>


namespace RDBVehicleAPI {

template<class T>
class TSQueue {
public:
	/**
	 * Type definition for callback function invoked when a new
	 * element is enqueued.
	 */
	typedef std::function<void (TSQueue<T>*)> CallbackType;

	/**
	 * Constructor
	 *
	 * @param maxSize The max size of the queue.
	 */
	TSQueue(uint32_t maxSize = 32) :
		maxSize_(maxSize)
	{
	};

	/**
	 * Destructor
	 */
	virtual ~TSQueue()
	{
		clear();
	};

	/**
	 * clear
	 */
	void clear()
	{
        std::lock_guard<std::mutex> lock(mutex_);
        for (uint i = 0; i < queue_.size(); i++) {
			queue_.pop();
		}
		callbackList_.clear();
	};
	/**
	 * If the queue is empty.
	 *
	 * @return True if empty, false otherwise.
	 */
	bool empty() const {
		return queue_.empty();
	};

	/**
	 * Get the maximum size of the queue.
	 *
	 * @return The maximum size of the queue.
	 */
	int getMaxSize() const {
		return maxSize_;
	};

	/**
	 * Get the size of the queue.
	 *
	 * @return The size (number of elements) of the queue.
	 */
	int size() const {
		return queue_.size();
	};

	/**
	 * Push a new element into the queue.
	 *
	 * @param element The element to be enqueued.
	 * @return True on success, or false if the queue is already
	 *         full.
	 *
	 * Note The callback function will be invoked
	 * here if it is registered.
	 */
	bool enqueue(std::shared_ptr<T> element) {
		bool rtn = true;
		std::lock_guard<std::mutex> lock(mutex_);
		if (queue_.size() == maxSize_) {
			rtn = false;
		} else {
			queue_.push(element);
			for (auto callback : callbackList_) {
				if (callback != nullptr) {
					(*callback)(this);
				}
			}
		}

		return rtn;
	};

	/**
	 * Pop the first element of the queue.
	 *
	 * @return Return the first element of the queue if the
	 *         queue is not empty, or return null.
	 */
	std::shared_ptr<T> dequeue() {
		std::shared_ptr<T> rtn = std::shared_ptr<T>((T*)nullptr);
		std::lock_guard<std::mutex> lock(mutex_);
		if (queue_.size() > 0) {
			rtn = queue_.front();
			queue_.pop();
		}

		return rtn;
	};

	/**
	 * Register enqueue listener callback function which
	 * will be invoked at enqueue().
	 *
	 * @param callback The callback function.
	 *
	 * Note the callback function is normally used to notify
	 * the thread which is waiting on the queue.
	 */
	void addEnqueueListener(CallbackType* callback)
	{
        std::lock_guard<std::mutex> lock(mutex_);
		callbackList_.push_back(callback);
	}

	/**
	 * Clear all enqueue listener callback functions.
	 */
	void clearEnqueueListener()
	{
        std::lock_guard<std::mutex> lock(mutex_);
		callbackList_.clear();
	}

private:
	TSQueue(const TSQueue& queue);
	TSQueue& operator= (const TSQueue& queue);

	std::queue<std::shared_ptr<T>> queue_;
	std::mutex mutex_;
    uint maxSize_;
	std::list<CallbackType*> callbackList_;
};

} /* namespace util */

#endif /* COM_YGOMI_ROADDB_UTIL_TSQUEUE_HPP_ */
