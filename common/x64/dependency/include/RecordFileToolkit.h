/**
*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG. 2016-2017
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
******************************************************************************
* @file   RecordFileToolkit.h
* @brief  Declaration of RecordFileToolkit
*******************************************************************************
*/
#ifndef RECORD_FILE_TOOLKIT_H
#define RECORD_FILE_TOOLKIT_H
#include <string>
#include <stdint.h>

namespace RecordFileToolkit
{
/**
 *******************************************************************************
 * @brief NDS2Degree 
 *
 *  <1> Parameter Description:
 *
 *  @param 
 *   nds [IN] nds 
 *
 *
 *  @return 
 *   Degree value
 *
 *  <2> Detailed Description:
 *  NDS2Degree 
 *******************************************************************************
 */

double NDS2Degree(int32_t nds);

/**
 *******************************************************************************
 * @brief Degree2NDS 
 *
 *  <1> Parameter Description:
 *
 *  @param 
 *  degree [IN] nds 
 *
 *
 *  @return 
 *   nds value
 *
 *  <2> Detailed Description:
 *  Degree2NDS 
 *******************************************************************************
 */

int32_t Degree2NDS(double degree);

/**
 *******************************************************************************
 * @brief toRadians 
 *
 *  <1> Parameter Description:
 *
 *  @param 
 *    angdeg [IN] angdeg 
 *
 *
 *  @return 
 *  Radians value
 *
 *  <2> Detailed Description:
 *  toRadians 
 *******************************************************************************
 */


double toRadians(double angdeg);

/**
 *******************************************************************************
 * @brief toGPS 
 *
 *  <1> Parameter Description:
 *
 *  @param 
 *
 *      refLng [IN] refLng
 *      refLat [IN] refLat
 *      x [IN] x value
 *      y [IN] y value
 *      outLng [OUT] outLng 
 *      outLat [OUT] outLat
 *
 *
 *  @return 
 *
 *  <2> Detailed Description:
 *  toGPS 
 *******************************************************************************
 */


void toGPS(double refLng, double refLat, double x, double y, double &outLng, double &outLat);

void toRelative(double refLng, double refLat, double lng, double lat, double &x, double &y);

void initRadius(double refLat);

void abs2Rel(double refLng, double refLat, double lng, double lat, double &x, double &y);

void rel2Abs(double refLng, double refLat, double x, double y, double &outLng, double &outLat);

bool transform(const std::string &irtv_path, const std::string &ortv_path, uint16_t start_index, uint16_t num = static_cast<uint16_t> (0), bool del_irtv = true);

bool getJPEGResolution(const char* jpegbuf, uint32_t jpegsize, uint16_t *ptrW, uint16_t *ptrH);

bool merge(const std::string &ipath_head, const std::string &ipath_tail, const std::string &opath, uint32_t range_of_time_diff = 1000/*ms*/);

}

#endif

