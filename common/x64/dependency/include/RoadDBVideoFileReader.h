/**
*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG. 2018-2019
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
******************************************************************************
* @file   RoadDBVideoFileReader.h
* @brief  Implementation of RoadDBVideoFileReader 
*******************************************************************************
*/
#ifndef ROADDBVIDEOFILEREADER_H
#define ROADDBVIDEOFILEREADER_H 
#include "RoadDBVideoErrMsg.h"
#include "IRoadDBVideoFile.h"

#include <memory>


class RoadDBVideoFileReader : public IRoadDBVideoFile 
{
    public:
        RoadDBVideoFileReader(const std::string &videoFile);
        virtual ~RoadDBVideoFileReader();
        /**
         *******************************************************************************
         * @brief getVersion 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  ver [OUT] the version of roaddb video 
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  get the version of roaddb video 
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E getVersion(ROAD_DB_VERSION_E &ver) = 0;

        /**
         *******************************************************************************
         * @brief getTimeFlag 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  index [IN] index of frame   
         *  usec [OUT] the index Relative time of frame , The unit is microsecond. 
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  get the index  Relative time of frame , The unit is microsecond. 
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E getTimeFlag(uint32_t index, uint64_t &usec) = 0;
        
        /**
         *******************************************************************************
         * @brief getTimeStamp 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  index [IN] index of frame   
         *  usec [OUT] the index Absolutely time of frame , The unit is microsecond. 
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  get the index Absolutely time of frame , The unit is microsecond. 
         *******************************************************************************
         */

        virtual ROAD_DB_ERROR_CODE_E getTimeStamp(uint32_t index, uint64_t &usec) = 0;
        /**
         *******************************************************************************
         * @brief getRawImageSize 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  index [IN] index of frame   
         *  size [OUT] the size of raw image 
         *  checkCorruption [IN] if check coruption 
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  get the image size
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E getRawImageSize(uint32_t index, uint32_t &size, bool checkCorruption = true) = 0;

        /**
         *******************************************************************************
         * @brief getRawImage 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  index [IN] index of frame   
         *  bufSize [IN] the size of the buf 
         *  buf [OUT] the data of image 
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  get the image data
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E getRawImage(uint32_t index, uint32_t bufSize, uint8_t *buf) = 0;

        /**
         *******************************************************************************
         * @brief getRawImage 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  index [IN] index of frame   
         *  cnt [OUT] the data of image 
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  get the image data
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E getRawImage(uint32_t index, std::string &cnt) = 0;
 

        virtual ROAD_DB_ERROR_CODE_E getImageSizeInBytes(uint32_t &size, ROAD_DB_FORMAT_E format = ROAD_DB_FORMAT_BGR_E, SCALE_FACTOR_E factor = SCALE_FACTOR_NO_E, uint16_t cutTop = 0, uint16_t cutBottom = 0, uint16_t *scaledWidth = NULL, uint16_t *scaledHeight = NULL) = 0;
        virtual ROAD_DB_ERROR_CODE_E getImage(uint32_t index, uint32_t bufSize, unsigned char *buf, ROAD_DB_FORMAT_E format = ROAD_DB_FORMAT_BGR_E, SCALE_FACTOR_E factor = SCALE_FACTOR_NO_E, uint16_t cutTop = 0, uint16_t cutBottom = 0) = 0;
    
        virtual ROAD_DB_ERROR_CODE_E  getImageColorFormat(ROAD_DB_FORMAT_E & format) = 0;

        /**
         *******************************************************************************
         * @brief getFramesCount 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  count [OUT] the frame count of roaddb video 
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  get the frame count of roaddb video 
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E getFramesCount(uint32_t &count) = 0;

        /**
         *******************************************************************************
         * @brief getFramesMax 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  count [OUT] the max frame count of roaddb video 
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  get the max frame count of roaddb video 
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E getFramesMax(uint32_t &count) = 0;

        /**
         *******************************************************************************
         * @brief getImageResolution 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  width [OUT] the pic width of the video 
         *  height [OUT] the pic height of the video
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  get resolution  
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E getImageResolution(uint16_t &width, uint16_t &height) = 0;

        /**
         *******************************************************************************
         * @brief getResolution 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  format [OUT] the pic format eg. jpg 
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  get pic format 
         *******************************************************************************
         */

        virtual ROAD_DB_ERROR_CODE_E getColorFormat(ROAD_DB_FORMAT_E & format) = 0;
        
        /**
         *******************************************************************************
         * @brief getResolution 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  codec [OUT] the pic codec eg. h264 
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  get pic codec 
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E getCodec(ROAD_DB_CODEC_E & codec) = 0;


        /**
         *******************************************************************************
         * @brief getStartTime 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  usec [OUT] the start time of frame , The unit is microsecond. 
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  get the start time of frame , The unit is microsecond.
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E getStartTime(uint64_t &usec) = 0;


        virtual ROAD_DB_ERROR_CODE_E getArrivedTime(uint32_t index,uint64_t &usec) = 0;

        virtual ROAD_DB_ERROR_CODE_E getSensorTime(uint32_t index,uint64_t &usec) = 0;

        /**
         *******************************************************************************
         * @brief getDeviceId 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  id [OUT] the device id 
         *  arraySize [IN] the size of buffer 
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  get the device id
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E getDeviceId(uint8_t id[], uint32_t arraySize) = 0;

        /**
         *******************************************************************************
         * @brief getUndistortFlags 
         *
         *  <1> Parameter Description:
         *
         *  @param 
         *  flags [OUT] the flags of undistort 
         *
         *
         *  @return 
         *  ROAD_DB_ERROR_CODE_SUCCESS_E ok
         *
         *  <2> Detailed Description:
         *  get the flags of undistort
         *******************************************************************************
         */
        virtual ROAD_DB_ERROR_CODE_E getUndistortFlags(uint32_t &flags) = 0;
        
 
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
        virtual std::string getCameraParams() = 0;

        virtual ROAD_DB_ERROR_CODE_E loadAllFromFile() = 0;

        virtual std::string getMetaInfo() = 0;

        /**
         *******************************************************************************
         * @brief getExtendedInfo 
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
        virtual  std::string getExtendedInfo() = 0;

       
};


std::shared_ptr<RoadDBVideoFileReader> makeRoadDBVideoFileReader(const std::string &videoFile);  

#endif
