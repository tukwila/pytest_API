/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   dataCompatibility.h
 * @brief  Head file of data compatibility interface.
 *******************************************************************************
 */

#ifndef DATA_COMPATIBILITY_H
#define DATA_COMPATIBILITY_H

#include "typeDef.h"



namespace roadDBCore
{
struct VehicleDivisionDetail_t;

namespace rdbSerialization
{
class  RdbV2SBinDeserializer;

uint32_t deseriaCompatibleData(RdbV2SBinDeserializer &rdbDeserializer, uint32_t version,
                               VehicleDivisionDetail_t &vehicleDetail);







} // namespace rdbSerialization

} // namespace roadDBCore




#endif
