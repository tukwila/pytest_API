/**
*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG. 2018-2019
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
******************************************************************************
* @file   IRoadDBVideoFile.h
* @brief  Implementation of IRoadDBVideoFile 
*******************************************************************************
*/
#ifndef IROADDBVIDEOFILE_H
#define IROADDBVIDEOFILE_H
#include <string>
#include <memory>

#include "RoadDBVideoErrMsg.h"
#include "RoadDBVideoCommDef.h"
#define DISTORTION_CORRECTED 0x0010

class IRoadDBVideoFile 
{
    public:
        IRoadDBVideoFile(const std::string videoFile);
        virtual ~IRoadDBVideoFile();
        

        
        
        /**
         *******************************************************************************
         * @brief open 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  open a video file use default path  
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E open() = 0;
        
        /**
         *******************************************************************************
         * @brief close 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  close a video 
         *******************************************************************************
         */
        virtual void close() = 0;

        virtual std::string getFileName();

    protected:
        std::string filePath_;
};




#endif
