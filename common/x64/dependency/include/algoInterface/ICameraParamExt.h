/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ICameraParamExt.h
 * @brief  file of extrinsic camera parameter interface
 *******************************************************************************
 */

#ifndef ICAMERA_PARAM_EXT_H_
#define ICAMERA_PARAM_EXT_H_

#include "typeDef.h"

namespace algo
{
  /**
    *******************************************************************************
    * @class ICameraParamExt ICameraParamExt.h
    * @brief interface of extrinsic camera Param
    * some definition:
    *******************************************************************************
    */
  using roadDBCore::float32_t;
  class C_ICameraParamExt
  {
  public:
      virtual ~C_ICameraParamExt() {};

      /**
      *******************************************************************************
      * @brief rl_X - get X
      *
      *  <1> Parameter Description:
      *
      *  @return status - return x.
      *                   x = distance from ground to camera, vehicle forward direction, unit: m
      *
      *  <2> Detailed Description:
      *      default value: 0
      *  \ingroup
      *******************************************************************************
      */
    virtual float32_t rl_X(void) const  = 0;

    /**
      *******************************************************************************
      * @brief rl_Y - get Y
      *
      *  <1> Parameter Description:
      *
      *  @return status - return y.
      *                   y = distance from ground to camera, vehicle left direction, unit: m
      *
      *  <2> Detailed Description:
      *      default value: 0
      *  \ingroup
      *******************************************************************************
      */
    virtual float32_t rl_Y(void) const = 0;

    /**
      *******************************************************************************
      * @brief rl_Z - get Z
      *
      *  <1> Parameter Description:
      *
      *  @return status - return z.
      *                   z = distance from ground to camera, vehicle up direction(camera height), unit: m
      *
      *  <2> Detailed Description:
      *      default value: 0
      *  \ingroup
      *******************************************************************************
      */
    virtual float32_t rl_Z(void) const = 0;

    /**
      *******************************************************************************
      * @brief rl_Roll - get Roll
      *
      *  <1> Parameter Description:
      *
      *  @return status - return roll
      *                   Roll = rotation angle between image vertical axis and world vertical axis (z axis) of the vehicle, around world's x axis, unit: rad.
      *
      *  <2> Detailed Description:
      *      default value: 0
      *  \ingroup
      *******************************************************************************
      */
    virtual float32_t rl_Roll(void) const = 0;

    /**
      *******************************************************************************
      * @brief rl_Pitch - get Pitch
      *
      *  <1> Parameter Description:
      *
      *  @return status - return pitch
      *                   pitch = rotation angle between image vertical axis and world vertical axis (z axis) of the vehicle, around world's y axis, unit: rad.
      *
      *  <2> Detailed Description:
      *      default value: 0
      *  \ingroup
      *******************************************************************************
      */
    virtual float32_t rl_Pitch(void) const = 0;

    /**
      *******************************************************************************
      * @brief rl_Yaw - get Yaw
      *
      *  <1> Parameter Description:
      *
      *  @return status - return yaw
      *                   yaw = rotation angle between image horizontal axis and world horizontal axis (y axis) of the vehicle, around z axis, unit: rad.
      *
      *  <2> Detailed Description:
      *      default value: 0
      *  \ingroup
      *******************************************************************************
      */
    virtual float32_t rl_Yaw(void) const = 0;
  };

}
#endif
