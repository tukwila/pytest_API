/**
*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG. 2018-2019
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
******************************************************************************
* @file   RoadDBVideoFileModifier.h
* @brief  Implementation of RoadDBVideoFileModifier 
*******************************************************************************
*/
#ifndef ROADDBVIDEOMODIFIER_H
#define ROADDBVIDEOMODIFIER_H

#include "RoadDBVideoErrMsg.h"
#include "IRoadDBVideoFile.h"

#include <memory>

class RoadDBVideoFileModifier : public IRoadDBVideoFile 
{
    public:

        RoadDBVideoFileModifier(const std::string &videoFile);
        virtual ~RoadDBVideoFileModifier();

        /**
         *******************************************************************************
         * @brief modifyCameraID 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *
         *
         *  @return 
         *   
         *
         *  <2> Detailed Description:
         *  modify camera id 
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E modifyCameraID(const uint8_t id[], uint32_t arraysize) = 0; 

        /**
         *******************************************************************************
         * @brief modifyUndistorted 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *
         *
         *  @return 
         *   
         *
         *  <2> Detailed Description:
         *  modify undistorted  
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E modifyUndistorted(bool undistorted) = 0;
        
        /**
         *******************************************************************************
         * @brief modifyCameraParams  
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *
         *
         *  @return 
         *   
         *
         *  <2> Detailed Description:
         *  modify camera params 
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E modifyCameraParams(const std::string &cameraParams) = 0;
        
        /**
         *******************************************************************************
         * @brief modifyFrameIndexDataChecksum 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *
         *
         *  @return 
         *   
         *
         *  <2> Detailed Description:
         *  modify frame index data checksum 
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E modifyFrameIndexDataChecksum(const std::string & frameIndexDataChecksum) = 0;

        virtual ROAD_DB_ERROR_CODE_E modifyTimestamp(uint32_t index,uint64_t &usec) = 0;
        virtual ROAD_DB_ERROR_CODE_E modifyArrivalTime(uint32_t index,uint64_t &usec) = 0;
        virtual ROAD_DB_ERROR_CODE_E modifyStartTime(uint64_t &usec) = 0;

        /**
         *******************************************************************************
         * @brief modifyExtendedInfo 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *
         *
         *  @return 
         *   
         *
         *  <2> Detailed Description:
         *  get the camera params for undistort
         *******************************************************************************
         */
        virtual  ROAD_DB_ERROR_CODE_E modifyExtendedInfo(const std::string & extendedInfo) = 0;


        virtual ROAD_DB_ERROR_CODE_E correctVideoResolution() = 0;
};


std::shared_ptr<RoadDBVideoFileModifier> makeRoadDBVideoFileModifier(const std::string &videoFile);


#endif


