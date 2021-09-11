/**
 *******************************************************************************
 *                         RoadDB Confidential
 *                    Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
* @file   ContinuityDetection.cpp
* @brief  Implementation of ContinuityDetection
*******************************************************************************
*/

#include <jsoncpp.hpp>
#include <string>

/**************************************************************************//**
 @Function      void ContinuityDetect(const string &sInputDir, Value &vJValue)
 

 @Description   group the metas in sInputDir directory.

 @Param[in]     sInputDir
                    the diretory inlcude meta files input.

 @Param[out]     vJValue
                    return group result.

 @Param[in]     lVehicleCount
                    intput the vehicle count.

 @Return        none.

 @Cautions      none.
*//***************************************************************************/
void ContinuityDetect(const std::string &sInputDir, Json::Value &vJValue, long lVehicleCount = 1);

