/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Runnable.h
 * @brief  This class wrapps a runnable function/method or a piece of code
 *         snippets into a class.
 *
 *         There are two ways to use this class:
 *         1. Attach a function directly to a runnable instance;
 *         2. Inherit this class and override the run() method.
 *
 * Rivision History:
 *      Date              Submitter         Description
 *      2018.02.05        Tony Xiong		Initialization
 *******************************************************************************
 */
#ifndef COM_YGOMI_ROADDB_UTIL_RUNNABLE_H_
#define COM_YGOMI_ROADDB_UTIL_RUNNABLE_H_

#include <functional>

namespace RDBVehicleAPI
{

class Runnable {

public:
	/**
	 * The signature of the callback function.
	 */
	typedef std::function<void ()> FuncType;

	/**
	 * Constructor
	 *
	 * @param func The callback function attached to this work.
	 */
	Runnable(FuncType* func = nullptr);


	/**
	 * Destructor
	 */
	virtual ~Runnable();

	/**
	 * The method to run this runnable.
	 *
	 * Note that once this method is overridden, the function attached
	 * will be ineffective, unless it is explicitly invoked in the
	 * overridden run().
	 */
	virtual void run();

private:
	FuncType* pFunc_;
};

// } /* namespace util */
} /* namespace roaddb */


#endif /* COM_YGOMI_ROADDB_UTIL_RUNNABLE_H_ */
