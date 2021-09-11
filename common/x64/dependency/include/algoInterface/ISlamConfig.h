/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ISlamConfig.h
 * @brief  file of orb slam2 config interface
 *******************************************************************************
 */
#ifndef ISLAM_CONFIG_H_
#define ISLAM_CONFIG_H_

#include "typeDef.h"

namespace algo
{

using roadDBCore::float32_t;
using roadDBCore::float64_t;
using roadDBCore::int32_t;

/**
 *******************************************************************************
 * @class ISlamConfig ISlamConfig.h
 * @brief interface of slam config
 *******************************************************************************
 */

class ISlamConfig
{
public:

    /**
     *******************************************************************************
     * Corner Parameters
     *******************************************************************************
     */

    virtual bool load(const std::string &strSettingFile, const std::string &rootName) {return false;}

    /**
     *******************************************************************************
     * @brief getFeatures - get features
     *
     *  <1> Parameter Description:
     *
     *  @return status - return number of features per image
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getFeatures(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getScaleFactor - get scale factor
     *
     *  <1> Parameter Description:
     *
     *  @return status - return scale factor between levels in the scale pyramid
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getScaleFactor(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getLevels - get levels
     *
     *  <1> Parameter Description:
     *
     *  @return status - return number of levels in the scale pyramid
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getLevels(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getIniThFAST - get IniThFAST
     *
     *  <1> Parameter Description:
     *
     *  @return status - return init Fast threshold
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getIniThFAST(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getMinThFAST - get MinThFAST
     *
     *  <1> Parameter Description:
     *
     *  @return status - return min Fast threshold
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getMinThFAST(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getScoreType - get score type
     *
     *  <1> Parameter Description:
     *
     *  @return status - return score type
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getScoreType(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getSUSANQualityLevel - get SUSAN qualityLevel parameter
     *
     *  <1> Parameter Description:
     *
     *  @return status - return SUSAN qualityLevel parameter
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float64_t getSUSANQualityLevel(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getSUSANMinDistance - get SUSAN minDistance parameter
     *
     *  <1> Parameter Description:
     *
     *  @return status - return SUSAN minDistance parameter
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float64_t getSUSANMinDistance(void) const  = 0;


    /**
     *******************************************************************************
     * SGD Parameters
     *******************************************************************************
     */

    /**
     *******************************************************************************
     * @brief getSGDScaleNum - get SGD scale number
     *
     *  <1> Parameter Description:
     *
     *  @return status - return scale number used by SGD.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getSGDScaleNum(void) const = 0;

    /**
     *******************************************************************************
     * @brief getSGDOrientationNum - get SGD Orientation number
     *
     *  <1> Parameter Description:
     *
     *  @return status - return orientation number used by SGD.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getSGDOrientationNum(void) const = 0;

    /**
     *******************************************************************************
     * @brief getSGDMatchThHigh - get SGD matching distance threshold of max ratio
     *
     *  <1> Parameter Description:
     *
     *  @return status - return matching distance threshold used by SGD.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getSGDMatchThHigh(void) const = 0;

    /**
     *******************************************************************************
     * @brief getSGDMatchThLow - get SGD matching distance min ratio of max threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return matching min distance threshold used by SGD.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getSGDMatchThLow(void) const = 0;


    /**
     *******************************************************************************
     * Tracker Parameters
     *******************************************************************************
     */

    /**
     *******************************************************************************
     * @brief getMinThOfKFGen - MinThOfKFGen
     *
     *  <1> Parameter Description:
     *
     *  @return status - return min frame threshold of key frame generate.
     *                   The threshold means how many frames need generate one key frame.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getMinThOfKFGen(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getMaxThOfKFGen - get MaxThOfKFGen
     *
     *  <1> Parameter Description:
     *
     *  @return status - return maximum frames threshold of key frame generate.
     *                   The threshold means how many frames need generate one key frame.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getMaxThOfKFGen(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getTrackingRefRatio - get tracking reference ratio
     *
     *  <1> Parameter Description:
     *
     *  @return status - return tracking reference ratio of SLAM.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getTrackingRefRatio(void) const = 0;

    /**
     *******************************************************************************
     * @brief getTrackingNoNewKFs - get threshold of no new KeyFrame insertion of
     *                              tracking
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of no new KeyFrame insertion of tracking
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getTrackingNoNewKFs(void) const = 0;

    /**
     *******************************************************************************
     * @brief getTrackingNewKFs - get threshold of new KeyFrame insertion of tracking
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of new KeyFrame insertion of tracking
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getTrackingNewKFs(void) const = 0;

    /**
     *******************************************************************************
     * @brief getMotionModelWndTh - get motion model window size threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of motion model window size.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getMotionModelWndTh(void) const = 0;

    /**
     *******************************************************************************
     * @brief getHarrisResponseTh - get Harris response threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of Harris response threshold.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float64_t getHarrisResponseTh(void) const = 0;

    /**
     *******************************************************************************
     * @brief getRadiusByViewingCosHigh - get radius by viewing angle high threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return high threshold of radius by viewing angle.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getRadiusByViewingCosHigh(void) const = 0;

    /**
     *******************************************************************************
     * @brief getRadiusByViewingCosLow - get radius by viewing angle low threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return low threshold of radius by viewing angle.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getRadiusByViewingCosLow(void) const = 0;

    /**
     *******************************************************************************
     * @brief getSearchForInitializationLevelTh - get initialization levels threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return low threshold of initialization levels.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getSearchForInitializationLevelTh(void) const = 0;


    /**
     *******************************************************************************
     * LocalMapper Parameters
     *******************************************************************************
     */

    /**
     *******************************************************************************
     * @brief getCovisibilityKFsNum - get covisibility KeyFrame number of new
     *                                MapPoint creation
     *
     *  <1> Parameter Description:
     *
     *  @return status - return covisibility KeyFrames threshold of new MapPoint
     *                   creation.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getCovisibilityKFsNum(void) const = 0;

    /**
     *******************************************************************************
     * @brief getMapPointCullingThObs - get map point culling observations threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return low threshold of map point culling observations.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getMapPointCullingThObs(void) const = 0;

    /**
     *******************************************************************************
     * @brief getMapPointCullingFoundRatioTh - get map point culling found ratio threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of map point culling found ratio.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getMapPointCullingFoundRatioTh(void) const = 0;

    /**
     *******************************************************************************
     * @brief getKeyFrameCullingThObs - get keyframe culling map point observations threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of keyframe culling map point observations.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getKeyFrameCullingThObs(void) const = 0;

    /**
     *******************************************************************************
     * @brief getSearchInNeighborsFirstNeighborsTh - get searchInNeighbors first neighbors threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of searchInNeighbors first neighbors.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getSearchInNeighborsFirstNeighborsTh(void) const = 0;

    /**
     *******************************************************************************
     * @brief getSearchInNeighborsSecondNeighborsTh - get searchInNeighbors second neighbors threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of searchInNeighbors second neighbors.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getSearchInNeighborsSecondNeighborsTh(void) const = 0;

    /**
     *******************************************************************************
     * @brief getCheckIsKeyFrameTooCloseTh - get threshold for checking if two keyframes are too close
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of CheckIsKeyFrameTooCloseTh().
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getCheckIsKeyFrameTooCloseTh(void) const = 0;


    /**
     *******************************************************************************
     * NN Ratio Parameters
     *******************************************************************************
     */

    /**
     *******************************************************************************
     * @brief getTrackMotionModelNNRatio - get track with motion model nn ratio threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of track with motion model nn ratio.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getTrackMotionModelNNRatio(void) const = 0;

    /**
     *******************************************************************************
     * @brief getTrackReferenceKeyFrameNNRatio - get track reference keyframe nn ratio threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of track reference keyframe nn ratio.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getTrackReferenceKeyFrameNNRatio(void) const = 0;

    /**
     *******************************************************************************
     * @brief getTrackLocalMapNNRatio - get track local map nn ratio threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of track local map nn ratio.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getTrackLocalMapNNRatio(void) const = 0;

    /**
     *******************************************************************************
     * @brief getTrackRelocalizationNNRatio - get track relocalization nn ratio threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of track relocalization nn ratio.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getTrackRelocalizationNNRatio(void) const = 0;

    /**
     *******************************************************************************
     * @brief getTrackRelocalizationNNRatio2 - get track relocalization second nn ratio threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of track relocalization second nn ratio.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getTrackRelocalizationNNRatio2(void) const = 0;

    /**
     *******************************************************************************
     * @brief getMonocularInitializationNNRatio - get monocular initialization nn ratio threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of monocular initialization nn ratio.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getMonocularInitializationNNRatio(void) const = 0;

    /**
     *******************************************************************************
     * @brief getCreateNewMapPointsNNRatio - get create new map points nn ratio threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of create new map points nn ratio.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getCreateNewMapPointsNNRatio(void) const = 0;


    /**
     *******************************************************************************
     * Optimizer Parameters
     *******************************************************************************
     */

    /**
     *******************************************************************************
     * @brief getLocalBAOptimizeIterationNum - get local BA optimize iteration number threshold
     *
     *  <1> Parameter Description:
     *
     *  @return status - return threshold of local BA optimize iteration number.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getLocalBAOptimizeIterationNum(void) const = 0;


    /**
     *******************************************************************************
     * Viewer Parameters
     *******************************************************************************
     */

     /**
     *******************************************************************************
     * @brief getKeyFrameSize - get viewer keyframe size
     *
     *  <1> Parameter Description:
     *
     *  @return status - return viewer keyframe size.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getKeyFrameSize(void) const = 0;

    /**
     *******************************************************************************
     * @brief getKeyFrameLineWidth - get viewer keyframe line width
     *
     *  <1> Parameter Description:
     *
     *  @return status - return viewer keyframe line width.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getKeyFrameLineWidth(void) const = 0;

    /**
     *******************************************************************************
     * @brief getGraphLineWidth - get viewer graph line width
     *
     *  <1> Parameter Description:
     *
     *  @return status - return viewer graph line width.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getGraphLineWidth(void) const = 0;

    /**
     *******************************************************************************
     * @brief getPointSize - get viewer point size
     *
     *  <1> Parameter Description:
     *
     *  @return status - return viewer point size.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getPointSize(void) const = 0;

    /**
     *******************************************************************************
     * @brief getCameraSize - get viewer camera size
     *
     *  <1> Parameter Description:
     *
     *  @return status - return viewer camera size.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getCameraSize(void) const = 0;

    /**
     *******************************************************************************
     * @brief getCameraLineWidth - get viewer camera line width
     *
     *  <1> Parameter Description:
     *
     *  @return status - return viewer camera line width.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getCameraLineWidth(void) const = 0;

    /**
     *******************************************************************************
     * @brief getViewpointX - get viewer view point X
     *
     *  <1> Parameter Description:
     *
     *  @return status - return viewer view point X.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getViewpointX(void) const = 0;

    /**
     *******************************************************************************
     * @brief getViewpointY - get viewer view point Y
     *
     *  <1> Parameter Description:
     *
     *  @return status - return viewer view point Y.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getViewpointY(void) const = 0;

    /**
     *******************************************************************************
     * @brief getViewpointZ - get viewer view point Z
     *
     *  <1> Parameter Description:
     *
     *  @return status - return viewer view point Z.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getViewpointZ(void) const = 0;

    /**
     *******************************************************************************
     * @brief getViewpointF - get viewer view point F
     *
     *  <1> Parameter Description:
     *
     *  @return status - return viewer view point F.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getViewpointF(void) const = 0;

    /**
     *******************************************************************************
     * @brief parseFloat - get float value from configuration.
     *
     *  <1> Parameter Description:
     *  @param [In]  - param_name  Name of parammeter will be parsed.
     *
     *  @return Parsed value.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t parseFloat(const char* param_name) const = 0;

    /**
     *******************************************************************************
     * @brief parseInteger - get integer value from configuration.
     *
     *  <1> Parameter Description:
     *  @param [In]  - param_name  Name of parammeter will be parsed.
     *
     *  @return Parsed value.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t parseInteger(const char* param_name) const = 0;

    /**
     *******************************************************************************
     * @brief parseString - get string value from configuration.
     *
     *  <1> Parameter Description:
     *  @param [In]  - param_name  Name of parammeter will be parsed.
     *
     *  @return Parsed value.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual const char* parseString(const char* param_name) const = 0;

    /**
     *******************************************************************************
     * @brief addFloat - add float value to configuration.
     *
     *  <1> Parameter Description:
     *  @param [In]  - param_name  Name of parammeter will be added.
     *
     *  @param [In]  - v           Value of parammeter.
     *
     *  @return Parsed value.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual void addFloat(const char* param_name, float v) = 0;

    /**
     *******************************************************************************
     * @brief addInteger - add integer value to configuration.
     *
     *  <1> Parameter Description:
     *  @param [In]  - param_name  Name of parammeter will be added.
     *
     *  @param [In]  - v           Value of parammeter.
     *
     *  @return Parsed value.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual void addInteger(const char* param_name, int v) = 0;

    /**
     *******************************************************************************
     * @brief parseVector - parser float vector from configuration
     *
     *  <1> Parameter Description:
     *  @param [In]  - param_name  Name of parammeter will be added.
     *
     *  @param [Out]  - vOutValue           Value of parammeter.
     *
     *  @return Parsed value.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual void parseVector(const char* param_name, std::vector<float> & vOutValue) {};

    /**
     *******************************************************************************
     * @brief parseVector - parser int32_t vector from configuration
     *
     *  <1> Parameter Description:
     *  @param [In]  - param_name  Name of parammeter will be added.
     *
     *  @param [Out]  - vOutValue           Value of parammeter.
     *
     *  @return Parsed value.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual void parseVector(const char* param_name, std::vector<int32_t> & vOutValue) {};

    /**
     *******************************************************************************
     * @brief addString - add string value to configuration.
     *
     *  <1> Parameter Description:
     *  @param [In]  - param_name  Name of parammeter will be added.
     *
     *  @param [In]  - v           Value of parammeter.
     *
     *  @return Parsed value.
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual void addString(const char* param_name, const char* v) = 0;
};

}
#endif
