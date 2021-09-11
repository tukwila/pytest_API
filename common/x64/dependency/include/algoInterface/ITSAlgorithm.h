/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ITSAlgorithm.h
 * @brief  TS algorithm abstract interface file.
 *******************************************************************************
 */


#ifndef _ITS_ALGORITHM_H_
#define _ITS_ALGORITHM_H_

#include <string>

#include "ISignConfig.h"
#include "ITrafficSignParamIn.h"
#include "IRoadObject.h"
#include "algoInterface/algo_ts.h"

namespace algo
{

using namespace roadDBCore;

/**
 *******************************************************************************
 * @class ITSAlgorithm ITSAlgorithm.h
 * @brief interface of ITSAlgorithm
 *
 *  the class of traffic sign Algorithm
 *******************************************************************************
 */
class ITSAlgorithm
{
public:
    virtual ~ITSAlgorithm() {};
    ITSAlgorithm()
    {
        checkOpenclEnv();
    }
    void enableOpencl()
    {
        if (canUseOpencl_)
        {
            openclEnabled_ = true;
        }
    }
    void disableOpencl()
    {
        openclEnabled_ = false;
    }

    /**
    *******************************************************************************
    * @brief getInitStatus - get the algorithm initialization status
    *
    *  <1> Parameter Description:
    *  @param OUT
    *
    *  @return the status:
    *       0: uninitialized,
    *       1: objects in algorithm initialized
    *       2: required parameters initialized legally
    *
    *  <2> Detailed Description:
    *
    *  in group algorithmInterface
    *******************************************************************************
    */
    virtual int32_t getInitStatus() = 0;

    /**
    *******************************************************************************
    * @brief trafficSignProcess - process the traffic sign with input config and params
    *
    *  <1> Parameter Description:
    *  @param IN algo::ISignConfig *pSignConfig - ISignConfig pointer
    *         IN algo::ITrafficSignParamIn *pTSParamIn - ITrafficSignParamIn pointer
    *         OUT algo::ITrafficSignOut *pTSOut - ITrafficSignOut pointer
    *
    *  @return the result of process
    *
    *  <2> Detailed Description:
    *
    *  in group algorithmInterface
    *******************************************************************************
    */
    virtual int32_t trafficSignProcess(IN algo::ISignConfig *pSignConfig,
                                       IN algo::ITrafficSignParamIn *pTSParamIn,
                                       OUT algo::IRoadObjectOut *pTSOut) = 0;

    /**
    *******************************************************************************
    * @brief tsVisDebugEnable - enable debug
    *
    *  <1> Parameter Description:
    *  @param IN const std::string& resDir - resource path
    *
    *  @return result - the result of enabling
    *
    *
    *  <2> Detailed Description:

    *  \ingroup
    *******************************************************************************
    */
    virtual bool TSVisDebugEnable(IN const std::string& resDir) = 0;

    /**
    *******************************************************************************
    * @brief tsVisDebugSaveView - save traffic signs to a file by ViewInterface
    *
    *  <1> Parameter Description:
    *  std::string& viewFilePath - the file path to save views
    *
    *  @return result - the result of saveing
    *
    *
    *  <2> Detailed Description:

    *  \ingroup
    *******************************************************************************
    */
    virtual int32_t TSVisDebugSaveView(IN std::string& viewFilePath) = 0;

    //For SDOR: detect traffic sign in ROIs
    /**
    *******************************************************************************
    * @brief detectTSfromROI - detect traffic sign from dedicated ROIs
    *
    *  <1> Parameter Description:
    *  std::vector<SDORFrameInfo> &vSdorFrame - the frame info from SDOR which contains ROIs
    *  algo::IImage *pImages - the IImage pointer to get the image
    *  std::vector<SDORSign> &vSdorSign - the output structure which contains the detected and recognized sign
    *
    *  @return result - the result of detection and recognition
    *
    *
    *  <2> Detailed Description:

    *  \ingroup
    *******************************************************************************
    */
    virtual int32_t detectTSfromROI(IN const std::vector<algo::SDORFrameInfo> &vSdorFrame,
                                    IN std::shared_ptr<RoadDBVideoFileReader> pImages,
                                    OUT std::vector<algo::SDORSign> &vSdorSign) = 0;

    //initialize the processing with input parameters
    virtual int32_t initProc(IN const algo::tsImageCamParam_t &tsImgParam) = 0;

    // detect traffic sign based on input image and RT
    // call initProc() to transfer parameters first before detectTS()
    virtual int32_t detectTS(OUT std::vector<algo::SDORSign> &vSdorSign) = 0;
    //end For SDOR

protected:
    void checkOpenclEnv()
    {
        #ifdef OPENCL_ENABLED
            canUseOpencl_ = true;
            enableOpencl();
        #else
            canUseOpencl_ = false;
            disableOpencl();
        #endif
    }

protected:
    bool openclEnabled_ = false;
    bool canUseOpencl_ = false;

};

}
#endif

