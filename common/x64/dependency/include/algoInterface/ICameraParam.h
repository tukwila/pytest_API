/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ICameraParam.h
 * @brief  file of camera parameter interface
 *******************************************************************************
 */

#ifndef ICAMERA_PARAM_H_
#define ICAMERA_PARAM_H_

#include "typeDef.h"

namespace algo
{
/**
 *******************************************************************************
 * @class ICameraParam ICameraParam.h
 * @brief interface of camera Param
 * some definition:
 * Focal Length =  distance between the lens and the image sensor.
 * Scale Factors = pixels per real life measurement. That is, when the image is projected onto the image plane,
 *                 it is how many pixels are equal to 1 meter (or mm, or cm).
 * Skew Coefficient = This is the skew between the x and y axis. That is, if the angle between the x and y axis is not 0,
 *                 it has some sort of skew. It is usually close to 0 and is often ignored.
 * Principal Point =  What point on the camera sensor (in pixels) is the center of the image in the real world.
 *                 it is usually at the center of the image sensor.
 * Radial Distortion = distortion that increases as you go further from the center.
 * Tangential Distortion = Distortion caused when the lens is not parallel with the camera sensor.
 *******************************************************************************
 */
  using roadDBCore::float32_t;
class ICameraParam
{
public:
    virtual ~ICameraParam() {};

    /**
     *******************************************************************************
     * @brief fx - get fx
     *
     *  <1> Parameter Description:
     *
     *  @return status - return fx.
     *                   fx = F*s_x = Focal length * scale factors on x direction
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t fx(void) const  = 0;

    /**
     *******************************************************************************
     * @brief fy - get fy
     *
     *  <1> Parameter Description:
     *
     *  @return status - return fy.
     *                   fy = F*s_y = Focal length * scale factors on y direction
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t fy(void) const  = 0;

    /**
     *******************************************************************************
     * @brief cx - get cx
     *
     *  <1> Parameter Description:
     *
     *  @return status - return cx.
     *                   principal point/ optical center
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t cx(void) const  = 0;

    /**
     *******************************************************************************
     * @brief cy - get cy
     *
     *  <1> Parameter Description:
     *
     *  @return status - return cy.
     *                   principal point/ optical center
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t cy(void) const  = 0;

    /**
     *******************************************************************************
     * @brief k1 - get k1
     *
     *  <1> Parameter Description:
     *
     *  @return status - return k1.
     *                   Radial distortion coefficient 1
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t k1(void) const  = 0;

    /**
     *******************************************************************************
     * @brief k2 - get k2
     *
     *  <1> Parameter Description:
     *
     *  @return status - return k2.
     *                   Radial distortion coefficient 2
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t k2(void) const  = 0;

    /**
     *******************************************************************************
     * @brief k3 - get k3
     *
     *  <1> Parameter Description:
     *
     *  @return status - return k3.
     *                   Radial distortion coefficient 3
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t k3(void) const  = 0;

    /**
     *******************************************************************************
     * @brief p1 - get p1
     *
     *  <1> Parameter Description:
     *
     *  @return status - return p1.
     *                   Tangential distortion coefficient 1
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t p1(void) const  = 0;

    /**
     *******************************************************************************
     * @brief p2 - get p2
     *
     *  <1> Parameter Description:
     *
     *  @return status - return p2.
     *                   Tangential distortion coefficient 1
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t p2(void) const  = 0;

    virtual float32_t scalefactor(void) const { return 1.0f; }

    virtual float32_t invscalefactor(void) const { return 1.0f; }

    virtual bool setScale(float scale) {return false;}
};

}
#endif
