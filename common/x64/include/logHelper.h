/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   log_helper.h
 * @brief  helper for log struct
 *         one limit for use: must put a constant string before the new type
 *         for example LOG_INFO << "Landscape" << landscape;
 *******************************************************************************
 */
#pragma once

#include "commonLogHelper.h"
#include "Line.h"
#include "NURBSCurve.h"
#include "EVP.h"
#include "Lane.h"
#include "Junction.h"
#include "Landscape.h"
#include "Conf.h"

namespace RDBVehicleAPI
{  
//    LogWrapper &operator<<(LogWrapper& os, const dbRowSeq_t& data);
   LogWrapper &operator<<(LogWrapper& os, const refreshConf_t& data);
   LogWrapper &operator<<(LogWrapper& os, const conf_t& data);
   LogWrapper &operator<<(LogWrapper& os, const threadAttribute_t& data);
   LogWrapper &operator<<(LogWrapper& os, const NURBSCurve& data);
   LogWrapper &operator<<(LogWrapper &os, const Line& data);
   LogWrapper &operator<<(LogWrapper& os, const EVP& data);
   LogWrapper &operator<<(LogWrapper& os, const Lane& data);
   LogWrapper &operator<<(LogWrapper& os, const Junction& data);
   LogWrapper &operator<<(LogWrapper& os, const Node& data);
   LogWrapper &operator<<(LogWrapper& os, const Landscape& data);
}
