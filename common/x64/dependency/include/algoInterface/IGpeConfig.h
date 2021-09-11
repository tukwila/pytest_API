/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IGpeConfig.h
 * @brief  file of ground plane estimation interface
 *******************************************************************************
 */


#ifndef IGPECONFIG_H_
#define IGPECONFIG_H_

#include "typeDef.h"

using roadDBCore::float32_t;
using roadDBCore::int32_t;

namespace algo
{
class IGpeConfig
{
public:
    /**
     *******************************************************************************
     * @brief getOutputDir - get the folder path for all GPE's outputs
     *
     *  <1> Parameter Description:
     *
     *  @return status - return the folder path for all GPE's outputs
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual std::string getOutputDir(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getCameraHeight - get camera's height (meters) from ground plane
     *
     *  <1> Parameter Description:
     *
     *  @return status - return the height from ground plane
     *
     *
     *  <2> Detailed Description:
     *  if the returned value <= 0,  then GPE will calculate the height automatically

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getCameraHeight(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getRoiColStartRatio - get road roi column start ration of image columns
     *
     *  <1> Parameter Description:
     *
     *  @return status - return the roi start column ration of image columns.
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getRoiColStartRatio(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getRoiRowStartRatio - get road roi row start ration of image rows
     *
     *  <1> Parameter Description:
     *
     *  @return status - return roi row start ration of image rows.
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getRoiRowStartRatio(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getRoicolRation - get road roi column ration with image columns
     *
     *  <1> Parameter Description:
     *
     *  @return status - return road roi column ration with image columns
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getRoicolRatio(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getRoiRowRation - get road roi row ration with image rows
     *
     *  <1> Parameter Description:
     *
     *  @return status - return road roi column ration with image rows.
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getRoiRowRatio(void) const  = 0;



    /**
     *******************************************************************************
     * @brief getCoplanarThreshold - get the threshold when the points is in the same
     *                               plane.
     *
     *  <1> Parameter Description:
     *
     *  @return status - return history estimaiton times of correct plane.
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getCoplanarThreshold(void) const  = 0;


   /**
     *******************************************************************************
     * @brief getRansacTimes - get the maximum ransac times.
     *
     *  <1> Parameter Description:
     *
     *  @return status - return ransac times.
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
     virtual int32_t getRansacTimes(void) const  = 0;

   /**
     *******************************************************************************
     * @brief getMinDist3PointsInplane - get the minimum distance between two 3d
     *                                   coplanar points.
     *
     *  <1> Parameter Description:
     *
     *  @return status - return the minimum distance between two 3d
     *                          coplanar points.
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
     virtual float32_t getMinDistCoplanar(void) const  = 0;

   /**
     *******************************************************************************
     * @brief getMinAngleCoplanar - get the minimum angle between two 3d
     *                                   coplanar points.
     *
     *  <1> Parameter Description:
     *
     *  @return status - return get the minimum angle between two 3d
     *                                   coplanar points.
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
     virtual float32_t getMinAngleCoplanar(void) const  = 0;
 /**
     *******************************************************************************
     * @brief getMatchedPointsEudlDist2 - get 2 matched points in different image
     *                                    distance     of      image.
     *
     *  <1> Parameter Description:
     *
     *  @return status - return distance in sub-pixel of 2 points.
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getMatchedPointsEudlDist2(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getMateched2PointsYDist - get 2 matched points distance of image in Y axis.
     *
     *  <1> Parameter Description:
     *
     *  @return status - return Y-axis distance in sub-pixel of 2 points.
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getMateched2PointsYDist(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getMinInlierNum - get the minimum number of inliers of two images.
     *
     *  <1> Parameter Description:
     *
     *  @return status - return minmum number of inliers.
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getMinInlierNum(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getMaxFeaturePointsNum - get the maximum number of feature points per
     *                                 image.
     *
     *  <1> Parameter Description:
     *
     *  @return status - return maximum number of feature points.
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getMaxFeaturePointsNum(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getOpticalWindowSize - get the half of the side length of the search
     *                               window.
     *
     *  <1> Parameter Description:
     *
     *  @return status - return maximum iteration number.
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getOpticalWindowSize(void) const  = 0;
};
}
#endif
