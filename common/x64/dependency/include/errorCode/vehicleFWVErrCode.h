/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   vehicleFWVErrorCode.h
 * @brief  vihicle modules's error code define.
 *******************************************************************************
 */

#ifndef VEHICLE_FRAMEWORK_ERROR_CODE_H
#define VEHICLE_FRAMEWORK_ERROR_CODE_H

#include "typeDef.h"
#include "errorCode/moduleMaskCode.h"


namespace roadDBCore
{

const uint32_t VEHICLE_UPLOADER_NET_FAILED	= VEHICLE_UPLOADER + 0X01;//network connect failed, or other network error
const uint32_t VEHICLE_UPLOADER_CA_ERROR	= VEHICLE_UPLOADER + 0X02;//Certificate error
const uint32_t VEHICLE_UPLOADER_DB_OPEN_FAILED	= VEHICLE_UPLOADER + 0X03;//db open error
const uint32_t VEHICLE_UPLOADER_DB_CF_ERROR	= VEHICLE_UPLOADER + 0X04;//db config error
const uint32_t VEHICLE_UPLOADER_PARAM_ERROR	= VEHICLE_UPLOADER + 0X05;//Command params parse error
const uint32_t VEHICLE_UPLOADER_PARAM_LOSS	= VEHICLE_UPLOADER + 0X06;//Params not enough
const uint32_t VEHICLE_UPLOADER_DIR_EMPTY	= VEHICLE_UPLOADER + 0X07;//Directory to upload is empty
const uint32_t VEHICLE_UPLOADER_DIR_NOT_EXIST	= VEHICLE_UPLOADER + 0X08;//Directory to upload not exist
const uint32_t VEHICLE_UPLOADER_ZIP_FAILED	= VEHICLE_UPLOADER + 0X09;//Zip file failed
const uint32_t VEHICLE_UPLOADER_FILE_OP_ERROR	= VEHICLE_UPLOADER + 0X0a;//File operation failed, such as open, getsize, remove, md5 getting and so on
const uint32_t VEHICLE_UPLOADER_CURL_ERROR	= VEHICLE_UPLOADER + 0X0b;//Cur lib function excute error, the detail info please see log.
const uint32_t VEHICLE_UPLOADER_SVR_DEAL_ERROR	= VEHICLE_UPLOADER + 0X0c;//Http Server deal this post upload request failed.
const uint32_t VEHICLE_UPLOADER_SINGLEFILE_NAME_EMPTY  = VEHICLE_UPLOADER + 0x0d;//The name of file uploaded is empty.
const uint32_t VEHICLE_UPLOADER_OTHER		= VEHICLE_UPLOADER + 0Xffff;//Other

const uint32_t VEHICLE_DOWNLOADER_NO_DOWNLOADTASK       = VEHICLE_DOWNLOADER + 0x01;//No download task need to be done.
const uint32_t VEHICLE_DOWNLOADER_STDINWATCH_START_FAIL = VEHICLE_DOWNLOADER + 0x02;//StdinWatch start fail, It's maybe failed to create watch thread.
const uint32_t VEHICLE_DOWNLOADER_DB_OPEN_FAILED        = VEHICLE_DOWNLOADER + 0x03;//The settings.db can't be open.
const uint32_t VEHICLE_DOWNLOADER_INPUTPARAM_ERR        = VEHICLE_DOWNLOADER + 0x04;//The downloader component boot parameter error.
const uint32_t VEHICLE_DOWNLOADER_UNKNOWN_EXCEPTION     = VEHICLE_DOWNLOADER + 0x05;//Exception, such as access db, access network.
const uint32_t VEHICLE_DOWNLOADER_RENAME_FAILED         = VEHICLE_DOWNLOADER + 0x06;//When the file has been downloaded but rename file failly.
const uint32_t VEHICLE_DOWNLOADER_PARSE_SOURCE_TYPE_FAILED    = VEHICLE_DOWNLOADER + 0x07;//Parse the json string failly when get source type.
const uint32_t VEHICLE_DOWNLOADER_CURL_COMMAND_EMPTY    = VEHICLE_DOWNLOADER + 0x08;//The curl command is empty.
const uint32_t VEHICLE_DOWNLOADER_CURL_FAILED           = VEHICLE_DOWNLOADER + 0x09;//The curl command failed.
const uint32_t VEHICLE_DOWNLOADER_MD5_ERR               = VEHICLE_DOWNLOADER + 0x0a;//The md5 & length of file has been downloaded just is wrong.
const uint32_t VEHICLE_DOWNLOADER_BATCH_SIZE_ERR        = VEHICLE_DOWNLOADER + 0x0b;//Batch id info to download task is wrong;
const uint32_t VEHICLE_DOWNLOADER_INSTALL_ERR           = VEHICLE_DOWNLOADER + 0x0c;//Install command failed;
const uint32_t VEHICLE_DOWNLOADER_PATH_CREATE_FAILED    = VEHICLE_DOWNLOADER + 0x0d;//destination file path not exist try to create ,but not successfully
const uint32_t VEHICLE_DOWNLOADER_DATA_ERR              = VEHICLE_DOWNLOADER + 0x0e;//Select data from db err;
const uint32_t VEHICLE_DOWNLOADER_OPTION_ERR            = VEHICLE_DOWNLOADER + 0x0f;//Option error;
const uint32_t VEHICLE_DOWNLOADER_UPDATE_DOWN_INFO_ERR  = VEHICLE_DOWNLOADER + 0x10;//Update download task info err;
const uint32_t VEHICLE_DOWNLOADER_NOT_SUPPORT_DOWNLOAD_SOURCE_TYPE  = VEHICLE_DOWNLOADER + 0x11; // Not support download sour type;
const uint32_t VEHICLE_DOWNLOADER_OTHER                 = VEHICLE_DOWNLOADER + 0xffff;//Other dowloader error



} //namespace roadDBCore


#endif //COMMON_SYS_ERROR_CODE_H
