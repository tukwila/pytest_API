/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ILaneModelConfig.h
 * @brief  file of 3D lane model interface
 *******************************************************************************
 */
#ifndef ILANEMODELCONFIG_H_
#define ILANEMODELCONFIG_H_

#include "typeDef.h"

namespace algo
{
class ILaneModelConfig
{

public:
    /**
    *******************************************************************************
    * @brief getModelscale - get the scale between pixel value and real distance(meter)
    *
    *  <1> Parameter Description:
    *
    *  @return status - return scale, no unit
    *
    *
    *  <2> Detailed Description:

    *  \ingroup
    *******************************************************************************
    */
    virtual roadDBCore::float32_t getModelScale(void) const  = 0;
    /**
     *******************************************************************************
     * @brief getModelresolution - get the minimum distance in the reconstructed world
     *
     *  <1> Parameter Description:
     *
     *  @return status - return the minimum distance, unit:metrer
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual roadDBCore::float32_t getModelResolution(void) const  = 0;

    /**
     *******************************************************************************
     * @brief getRoiRowStartRatio - get the minimum length of an item piece for modeling
     *
     *  <1> Parameter Description:
     *
     *  @return status - return the minimum length, unit: meter.
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual roadDBCore::float32_t getModelPieceLen(void) const  = 0;
};
}
#endif
