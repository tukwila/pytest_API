/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ThreadConfig.h
 * @brief  Implementation of lock free loop queue and thread safe loop queue.
 *******************************************************************************
 */

#include <pthread.h>
#include <sched.h>
#include <boost/thread/thread.hpp>
#include "LogWrapper/LogWrapper.h"
#include "typeDef.h"

#ifndef THREAD_CONFIG_H_
#define THREAD_CONFIG_H_

namespace roadDBCore
{


class ThreadConfig
{
public:
    ThreadConfig() = delete;
    ThreadConfig(const ThreadConfig&) = delete;
    ThreadConfig& operator=(const ThreadConfig&) = delete;

public:
    static bool initAttributes(boost::thread::attributes& attr, int32_t sched_policy, int32_t priority, const std::vector<int32_t>& vCpus);
    static bool initAttributes(pthread_attr_t& attr, int32_t sched_policy, int32_t priority, const std::vector<int32_t>& vCpus);

};


}

#endif
