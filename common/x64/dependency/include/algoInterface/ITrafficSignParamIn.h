/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ITrafficSign.h
 * @brief  interface of traffic sign
 *******************************************************************************
 */
#ifndef _ITRAFFIC_SIGN_PARAM_IN_H_
#define _ITRAFFIC_SIGN_PARAM_IN_H_

#include <vector>
#include <opencv2/imgproc.hpp>
#include "typeDef.h"

#include "RoadDBVideoFileReader.h"
#include "ICameraParam.h"
#include "ICameraParamExt.h"
#include "IRTMatrix.h"

namespace algo
{

using namespace roadDBCore;

/**
 *******************************************************************************
 * @class TrafficSignParamIn ITrafficSignParamIn.h
 * @brief interface of TrafficSignParamIn
 *
 *  the class to gather all the traffic sign input parameters
 *******************************************************************************
 */
class ITrafficSignParamIn
{
public:
    virtual ~ITrafficSignParamIn() {};

    /**
    *******************************************************************************
    * @brief createParams - createParams the parameters for traffic sign processing
    *
    *  <1> Parameter Description:
    *  @param IN const std::string& tagVideoFile - video file with gps info
    *         IN const std::string& cameraFile - camera config file
    *         IN const std::string& rtMatrixFile - RT matrix of the video
    *
    *  @return true if creating the parameters successfully, otherwise false
    *
    *  <2> Detailed Description:
    *
    *  in group algorithmInterface
    *******************************************************************************
    */
    virtual int32_t createParams(IN const std::string& tagVideoFile,
            IN const std::string& cameraFile,
            IN const std::string& rtMatrixFile) = 0;

    /**
    *******************************************************************************
    * @brief createParams - createParams the parameters for traffic sign processing
    *
    *  <1> Parameter Description:
    *  @param IN const std::string& videoFile - video file
    *         IN const std::string& gpsFile - gps data file
    *         IN const std::string& cameraFile - camera config file
    *         IN const std::string& rtMatrixFile - RT matrix of the video
    *
    *  @return true if creating the parameters successfully, otherwise false
    *
    *  <2> Detailed Description:
    *
    *  in group algorithmInterface
    *******************************************************************************
    */
    virtual int32_t createParams(IN const std::string& videoFile,
            IN const std::string& gpsFile,
            IN const std::string& cameraFile,
            IN const std::string& rtMatrixFile) = 0;

    /**
    *******************************************************************************
    * @brief get - get the parameters for traffic sign processing
    *
    *  <1> Parameter Description:
    *  @param IN std::shared_ptr<RoadDBVideoFileReader> &FileReader, image frames
    *         IN algo::ICameraParam &cameraParam, the camera parameters
    *         IN algo::C_ICameraParamExt &cameraParamExt, the extended camera parameters
    *         IN algo::IRTMatrixIn &rtMatrix, RT matrix of the image
    *
    *  <2> Detailed Description:
    *
    *  in group algorithmInterface
    *******************************************************************************
    */
    virtual std::shared_ptr<RoadDBVideoFileReader>  getFileReader() = 0;
    virtual algo::ICameraParam* getCameraParam() = 0;
    virtual algo::C_ICameraParamExt* getCameraParamExt() = 0;
    virtual algo::IRTMatrixIn* getRTMatrixIn() = 0;
};
}
#endif
