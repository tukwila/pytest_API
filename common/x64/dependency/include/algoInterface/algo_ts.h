/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   algo_ts.h
 * @brief  algorithm traffic sign common header file.
 *******************************************************************************
 */

#ifndef ALGO_TS_H_
#define ALGO_TS_H_

#include <string>
#include <vector>
#include <opencv2/core/types.hpp>
#include "typeDef.h"
#include "algoInterface/ICameraParam.h"
#include "algoInterface/ICameraParamExt.h"
#include "algoInterface/IRTMatrix.h"
#include "RoadDBVideoFileReader.h"

namespace algo
{

struct tsConfigParam_t
{
    std::string tsConfigFile;      /* path of traffic sign config file  */
    std::string tsLogDir;          /* path of traffic sign log directory  */
};

struct tsImageCamParam_t
{
    std::shared_ptr<RoadDBVideoFileReader>  pIImg;
    algo::ICameraParam                     *pICamParam;
    algo::C_ICameraParamExt                *pCameraParamExt;
    algo::IRTMatrixIn                      *pIRTMat;
};

/**For SDOR begin**/
struct SDORFrameInfo
{
    bool                    bOutput;
    roadDBCore::int32_t     frameId;
    std::vector<cv::Rect>   vRoi;
};

struct SDORSignRoi
{
    roadDBCore::int32_t    frameId;        /** frame index containing detected sign ROI  */
    roadDBCore::int32_t    signType;       /*sign type*/
    roadDBCore::int32_t    signShape;      /*sign shape*/
    roadDBCore::float32_t  signConf;       /** sign confidence */
    cv::Rect               signRoi;        /** detected ROI in the 2D image */
};

struct SDORSign
{
    roadDBCore::int32_t       signRoiId;
    std::vector<SDORSignRoi>  vSdorSignRoi;
    /* the relative position in meter to the input coordinate
     * x axis towards North,
     * y axis towards East,
     * z axis toward  up */
    cv::Point3f               signPos;
};
/**For SDOR end**/

} /* namespace algo */

#endif /* ALGO_TS_H_ */
