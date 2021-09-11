/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ThreadsafeQueue.h
 * @brief
 *******************************************************************************
 */


#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>

#ifndef TREADSAFE_QUEUE_H
#define TREADSAFE_QUEUE_H

namespace roadDBCore
{

template<class T>
class ThreadsafeQueue
{
public:
	ThreadsafeQueue(){}

	ThreadsafeQueue(ThreadsafeQueue const& other)
	{
    	std::lock_guard<std::mutex> lk(other.mut);
    	data_queue = other.data_queue;
	}
/**
*******************************************************************************
* @brief push --- push the value to the queue
*
*  <1> Parameter Description:
*
*  @param [In] -- tValue  the data value 
*
*  <2> Detailed Description:
*******************************************************************************
*/
	void push(T tValue)
	{
    	std::lock_guard<std::mutex> lk(mut);
    	data_queue.push(tValue);
    	data_con.notify_one();
	}
/**
*******************************************************************************
* @brief wait_and_pop --- pop the element from the queue, if no element, wait
*
*  <1> Parameter Description:
*
*  @param [In] -- tValue  the data needed to pop
*
*  <2> Detailed Description:
*******************************************************************************
*/
	void wait_and_pop(T& tValue)
	{
    	std::unique_lock<std::mutex> lk(mut);
    	data_con.wait(lk,[this]{return !data_queue.empty();});
    	tValue = data_queue.front();
    	data_queue.pop();
	}
/**
*******************************************************************************
* @brief wait_and_pop --- pop the element from the queue, if no element, wait
*
*  <1> Parameter Description:
*
* @return the shared pointer of the data
*  <2> Detailed Description:
*******************************************************************************
*/
	std::shared_ptr<T>wait_and_pop()
	{
    	std::unique_lock<std::mutex> lk(mut);
    	data_con.wait(lk,[this]{return !data_queue.empty();});
    	std::shared_ptr<T> ret (std::make_shared<T>(data_queue.front()));
    	data_queue.pop();
    	return ret;
	}
/**
*******************************************************************************
* @brief try_pop --- pop the element from the queue
*
*  <1> Parameter Description:
* @param [In] -- tValue  the data needed to pop
*
* @return the true if queue not empty and pop the data, otherwise return false
*  <2> Detailed Description:
*******************************************************************************
*/
	bool try_pop(T& tValue)
	{
    	std::lock_guard<std::mutex> lk(mut);

    	if(data_queue.empty())
        {
            return false;
        }

    	tValue = data_queue.front();
    	data_queue.pop();
    	return true;
	}
/**
*******************************************************************************
* @brief clear --- clear all element from the queue
*
*  <1> Parameter Description:
*
*  <2> Detailed Description:
*******************************************************************************
*/
    void clear()
    {
        std::lock_guard<std::mutex> lk(mut);

        while(!data_queue.empty())
        {
            data_queue.pop();
        }
    }
/**
*******************************************************************************
* @brief try_pop --- pop the element from the queue
*
*  <1> Parameter Description:
* @param [In] -- tValue  the data needed to pop
*
* @return the shared pointer of the data
*  <2> Detailed Description:
*******************************************************************************
*/
	std::shared_ptr<T> try_pop()
	{
    	std::lock_guard<std::mutex> lk(mut);

    	if(data_queue.empty())
        {
            return std::shared_ptr<T>();
        }

    	std::shared_ptr<T> ret(std::make_shared(data_queue.front()));
    	data_queue.pop();
    	return ret;
	}
/**
*******************************************************************************
* @brief empty --- check the empty of the queue or not
*
*  <1> Parameter Description:
*
*  <2> Detailed Description:
*******************************************************************************
*/
	bool empty() const
	{
    	std::lock_guard<std::mutex> lk(mut);
    	return data_queue.empty();
	}
/**
*******************************************************************************
* @brief size --- check the size of the queue
*
*  <1> Parameter Description:
*
*  <2> Detailed Description:
*******************************************************************************
*/
    uint32_t size() const
    {
        std::lock_guard<std::mutex> lk(mut);
        return data_queue.size();
    }
/**
*******************************************************************************
* @brief returnEmptyAndPush --- check the empty of the queue or not and then push the data to the queue
*
*  <1> Parameter Description:
*
* @param [In] -- tValue  the data needed to push
* @return true if queue not empty, otherwise return false
*  <2> Detailed Description:
*******************************************************************************
*/
    bool returnEmptyAndPush(T tValue)
    {
        std::lock_guard<std::mutex> lk(mut);
        bool isEmpty = data_queue.empty();
        data_queue.push(tValue);
    	data_con.notify_one();

        return isEmpty;
    }



private:
	mutable std::mutex mut;
	std::queue<T> data_queue;
	std::condition_variable data_con;
};

}

#endif
