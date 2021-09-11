/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Work.h
 * @brief  This class represents a piece of work to do. It inherits Runnable
 *         and can be attached with an input message and an output message.
 *
 * Rivision History:
 *      Date              Submitter         Description
 *      2018.02.05        Tony Xiong		Initialization
 *******************************************************************************
 */
#ifndef COM_YGOMI_ROADDB_UTIL_WORK_H_
#define COM_YGOMI_ROADDB_UTIL_WORK_H_

#include <memory>
#include "Runnable.h"
#include "Message.h"


namespace RDBVehicleAPI
{

class Work: public Runnable {
public:
	Work(FuncType* func = nullptr);
	virtual ~Work();

	virtual void run();

	std::shared_ptr<Message> getInputMsg();
	std::shared_ptr<Message> getOutputMsg();

	void setInputMsg(std::shared_ptr<Message> inputMsg);
	void setOutputMsg(std::shared_ptr<Message> outputMsg);

private:
	std::shared_ptr<Message> _inputMsg;
	std::shared_ptr<Message> _outputMsg;
};

// } /* namespace util */
} /* namespace roaddb */


#endif /* COM_YGOMI_ROADDB_UTIL_WORK_H_ */
