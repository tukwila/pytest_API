/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ISdorConfig.h
 * @brief  file of SDOR configuration interface
 *******************************************************************************
 */

#pragma once
// local class header files

// system header files

// other lib header files

// local project header file
#include "typeDef.h"

// forward declaration


namespace algo
{

using roadDBCore::float32_t;
using roadDBCore::float64_t;
using roadDBCore::int32_t;

/**
 *******************************************************************************
 * @class ISdorConfig ISdorConfig.h
 * @brief interface of Sdor configuration interface
 *******************************************************************************
 */

class ISdorConfig
{
public:

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


    virtual bool load(const std::string &configFile) = 0;                                                                                                                                                       
    
    virtual int32_t getCutTop(void) const = 0;
    virtual int32_t getCutBottom(void) const = 0;
    virtual int32_t getImgScale(void) const = 0;

}; // class ISdorConfig

} // namespace algo
