/**
*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG. 2018-2019
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
******************************************************************************
* @file   RoadDBVideoErrMsg.h
* @brief  Implementation of RoadDBVideoErrMsg 
*******************************************************************************
*/
#ifndef ROADDBVIDEOERRMSG_H
#define ROADDBVIDEOERRMSG_H 
#include <string>
enum ROAD_DB_ERROR_CODE_E
{
	ROAD_DB_ERROR_CODE_SUCCESS_E = 0,
	ROAD_DB_ERROR_CODE_STATUS_FILE_CANNOT_READ_E,
	ROAD_DB_ERROR_CODE_FILE_NOT_EXIST_E,
	ROAD_DB_ERROR_CODE_INVALID_FILE_NAME_E,
	ROAD_DB_ERROR_CODE_NULL_POINTER_E,
	ROAD_DB_ERROR_CODE_FILE_SIZE_IS_ZERO_E,
	ROAD_DB_ERROR_CODE_FILE_OPEN_FAIL_E,
	ROAD_DB_ERROR_CODE_FILE_MMAP_FAIL_E,
	ROAD_DB_ERROR_CODE_NOT_ROADDB_VIDEO_FORMAT_E,
	ROAD_DB_ERROR_CODE_DUPLICATE_INIT_E,
	ROAD_DB_ERROR_CODE_NOT_INIT_E,
	ROAD_DB_ERROR_CODE_ALREADY_LAST_FRAME_E,
	ROAD_DB_ERROR_CODE_INVALID_PARAMETER_E,
	ROAD_DB_ERROR_CODE_DECOMPRESS_ERROR_E,
	ROAD_DB_ERROR_CODE_COLOR_FORMAT_NOT_SUPPROT_E,
	ROAD_DB_ERROR_CODE_BUFFER_NOT_ALLOCATED_E,
	ROAD_DB_ERROR_CODE_ALL_IMAGES_CORRUPTED_E,
	ROAD_DB_ERROR_CODE_VIDEO_NOT_COMPLETE_E,
	ROAD_DB_ERROR_CODE_OUT_RANGE_E,
	ROAD_DB_ERROR_CODE_MAX_E
};

std::string errorCode2String(ROAD_DB_ERROR_CODE_E errorCode);


#endif
