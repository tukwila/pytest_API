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
#ifndef I_SIGN_CONFIG_H_
#define I_SIGN_CONFIG_H_

#include "typeDef.h"

namespace algo
{

using roadDBCore::int32_t;


/**
 *******************************************************************************
 * @class ISignConfig ISignConfig.h
 * @brief interface of sign config
 *******************************************************************************
 */

class ISignConfig
{
public:

    /**
     *******************************************************************************
     * @brief getCountryCode - get country code
     *
     *  <1> Parameter Description:
     *
     *  @return status - return contry code, if failed, return -1;
     *
     *
     *  <2> Detailed Description:

     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getCountryCode(void) const  = 0;


    /**
    *******************************************************************************
    * @brief getResourceDir - get resource directory
    *
    *  <1> Parameter Description:
    *
    *  @return status - return resource directory.
    *
    *
    *  <2> Detailed Description:

    *  \ingroup
    *******************************************************************************
    */
   virtual std::string getResourceDir(void) const  = 0;

   virtual std::string getCES_TSR_Config(void) const  = 0;
};


}

#endif
