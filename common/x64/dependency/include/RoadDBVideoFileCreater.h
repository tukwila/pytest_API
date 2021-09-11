/**
*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG. 2018-2019
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
******************************************************************************
* @file   RoadDBVideoFileCreater.h 
* @brief  Implementation of RoadDBVideoFileCreater 
*******************************************************************************
*/
#ifndef ROADDBVIDEOFILECREATER_H
#define ROADDBVIDEOFILECREATER_H
#include <string>


#include "IRoadDBVideoFile.h"


class RoadDBVideoFileCreater : public IRoadDBVideoFile 
{
    public:
        RoadDBVideoFileCreater(const std::string videoFile);
        virtual ~RoadDBVideoFileCreater();
        /**
         *******************************************************************************
         * @brief setUndistortFlags 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  flags [IN] the flags of undistort 
         *
         *
         *  @return 
         *
         *  <2> Detailed Description:
         *  set the flags of undistort
         *******************************************************************************
         */
        virtual void setUndistortFlags() = 0;
        
        /**
         *******************************************************************************
         * @brief setDeviceId 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  flags [IN] the flags of undistort 
         *
         *
         *  @return 
         *
         *  <2> Detailed Description:
         *  set the flags of undistort
         *******************************************************************************
         */
        virtual void setDeviceId(const uint8_t id[], uint32_t arraysize) = 0;

        /**
         *******************************************************************************
         * @brief setStartTime 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  startTime [IN] the start time of frame 
         *
         *
         *  @return 
         *
         *  <2> Detailed Description:
         *  set start time of frame
         *******************************************************************************
         */
        virtual void setStartTime(uint64_t startTime);

        /**
         *******************************************************************************
         * @brief setVideoResolution 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  width [IN] the width of pic 
         *  height [IN] the height of pic 
         *
         *
         *  @return 
         *
         *  <2> Detailed Description:
         *  set the resolution of video
         *******************************************************************************
         */
        virtual void setVideoResolution(uint16_t width, uint16_t height) = 0; 

        /**
         *******************************************************************************
         * @brief setFrameFormat 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  format [IN] the format of pic 
         *
         *
         *  @return 
         *
         *  <2> Detailed Description:
         *  set the format of video
         *******************************************************************************
         */
        virtual void setFrameFormat(ROAD_DB_FORMAT_E format) = 0;

        /**
         *******************************************************************************
         * @brief setFrameCodec 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  codec [IN] the format of pic 
         *
         *
         *  @return 
         *
         *  <2> Detailed Description:
         *  set the codec of video
         *******************************************************************************
         */
        virtual void setFrameCodec(ROAD_DB_CODEC_E codec) = 0;

        /**
         *******************************************************************************
         * @brief addFrame 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  timeflag [IN] Relative time for video 
         *  framedata [IN] frame data
         *
         *
         *  @return 
         *
         *  <2> Detailed Description:
         *  add a frame in video with relative time
         *******************************************************************************
         */
        virtual bool addFrame(const std::string& frameData,uint64_t timeStamp, uint64_t arrivalTime) = 0;
        virtual bool addFrame(const uint8_t *frameData , uint32_t size,uint64_t timeStamp, uint64_t arrivalTime) = 0;

        /**
         *******************************************************************************
         * @brief getCameraParams 
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
        virtual  void setCameraParams(const std::string & cameraParams) = 0;

        /**
         *******************************************************************************
         * @brief setExtendedInfo 
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
        virtual  void setExtendedInfo(const std::string & extendedInfo) = 0;


};

std::shared_ptr<RoadDBVideoFileCreater> makeRoadDBVideoFileCreater(const std::string &videoFile); 
#endif 


