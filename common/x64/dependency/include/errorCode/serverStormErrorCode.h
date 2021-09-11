/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file  serverStormErrorCode.h
 * @brief Definition of error codes used by server-side of STORM.
 *******************************************************************************
 */
 
#ifndef SERVER_STORM_ERROR_CODE_H_
#define SERVER_STORM_ERROR_CODE_H_
 
#include "typeDef.h"
#include "errorCode/moduleMaskCode.h"

namespace roadDBCore
{

const uint32_t NULL_CONFIG_INSTANCE       = SERVER_STORM + 0x1;
const uint32_t NO_GEO_LOGIC_DIVISION      = SERVER_STORM + 0x2;
const uint32_t MEM_ALLOC_FAILURE          = SERVER_STORM + 0x3;
const uint32_t INNER_FLOW_EXECUTION_ERROR = SERVER_STORM + 0x4;

}
 
#endif // SERVER_STORM_ERROR_CODE_H_
