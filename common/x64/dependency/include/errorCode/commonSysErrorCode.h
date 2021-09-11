/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   commonSysErrorCode.h
 * @brief  common system error code define.
 *******************************************************************************
 */

#ifndef COMMON_SYS_ERROR_CODE_H
#define COMMON_SYS_ERROR_CODE_H

#include "typeDef.h"
#include "errorCode/moduleMaskCode.h"


namespace roadDBCore
{
const uint32_t RDB_SERIA_ERR_SAVE             = COMMON_SERIALIZATION + 0X1;
const uint32_t RDB_SERIA_ERR_LOAD             = COMMON_SERIALIZATION + 0X2;
const uint32_t RDB_SERIA_ERR_OPEN_FILE        = COMMON_SERIALIZATION + 0X3;
const uint32_t RDB_SERIA_ERR_BAD_STREAM       = COMMON_SERIALIZATION + 0X4;
const uint32_t RDB_LIB_MESSAGE                = COMMON_SERIALIZATION + 0X5;



// LOGIC DATA ACCESS
const uint32_t  REG_SERVER_MODULE             = COMMON_LIB_MESSAGE + 0X1;
const uint32_t  SESSION_NOT_EXIST             = COMMON_LIB_MESSAGE + 0X2;
const uint32_t  MOD_HANDLER_NOT_EXIST         = COMMON_LIB_MESSAGE + 0X3;
const uint32_t  START_SERVER_FAILD            = COMMON_LIB_MESSAGE + 0X4;
const uint32_t  MESSAGE_PRT_NULL              = COMMON_LIB_MESSAGE + 0X5;
const uint32_t  CLIENT_PORT_NOT_EXIST         = COMMON_LIB_MESSAGE + 0X6;
const uint32_t  SERVER_BUSY                   = COMMON_LIB_MESSAGE + 0X7;
const uint32_t  OPT_NOT_REGIST                = COMMON_LIB_MESSAGE + 0X8;
const uint32_t  MSG_LENGTH_LIMITED            = COMMON_LIB_MESSAGE + 0X9;
const uint32_t  MSG_PRT_NULL                  = COMMON_LIB_MESSAGE + 0X10;


const uint32_t  MSG_CLIENT_OK                 = 0;
const uint32_t  MSG_CLIENT_FAILED             = COMMON_LIB_MESSAGE + 0X31;
const uint32_t  MSG_CLIENT_PTR_NULL           = COMMON_LIB_MESSAGE + 0X32;
const uint32_t  MSG_CLIENT_PARAM_INVALID      = COMMON_LIB_MESSAGE + 0X33;
const uint32_t  MSG_CLIENT_NO_MEMORY          = COMMON_LIB_MESSAGE + 0X34;
const uint32_t  MSG_CLIENT_NOT_INITED         = COMMON_LIB_MESSAGE + 0X35;
const uint32_t  MSG_CLIENT_NOT_CONNECTED      = COMMON_LIB_MESSAGE + 0X36;
const uint32_t  MSG_CLIENT_START_ERR          = COMMON_LIB_MESSAGE + 0X37;
const uint32_t  MSG_CLIENT_SOC_READ_ERR       = COMMON_LIB_MESSAGE + 0X38;
const uint32_t  MSG_CLIENT_SOC_WRITE_ERR      = COMMON_LIB_MESSAGE + 0X39;
const uint32_t  MSG_CLIENT_SOC_TIMEOUT_ERR    = COMMON_LIB_MESSAGE + 0X40;
const uint32_t  MSG_CLIENT_UNKNOWN_ERR        = COMMON_LIB_MESSAGE + 0X60;








} //namespace roadDBCore


#endif //COMMON_SYS_ERROR_CODE_H
