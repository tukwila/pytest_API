/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ILanePaintConfig.h
 * @brief  file of lane detection and tracking parameter interface
 *******************************************************************************
 */
#ifndef ILANEPAINTCONFIG_H_
#define ILANEPAINTCONFIG_H_
#include "typeDef.h"

namespace algo
{
using roadDBCore::float32_t;
using roadDBCore::int32_t;

class ILanePaintConfig
{
public:
    /**
     *******************************************************************************
     * @brief getLaneWidthRation - get lane width ration with image width
     *
     *  <1> Parameter Description:
     *
     *  @return status - return lane width ratio (lane width / image width).
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getLaneWidthRation(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getHorizonLine - get horizon line location
     *
     *  <1> Parameter Description:
     *
     *  @return status - return horizon line location ratio in Y axis of image (horizon Y / image height).
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getHorizonLine(void) const  = 0;


    /**
     *******************************************************************************
     * @brief getBottomLine - get car window's Bottom line location, which is used for car-inside camera
     *
     *  <1> Parameter Description:
     *
     *  @return status - return bottome line location ratio in Y axis of image (bottom Y / image height).
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual float32_t getBottomLine(void) const  = 0;

};
}
#endif

