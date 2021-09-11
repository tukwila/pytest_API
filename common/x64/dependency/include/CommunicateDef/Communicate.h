/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Communicate.h
 * @brief  interface of communicate
 *******************************************************************************
 */

#ifndef _COMMUNICATE_H_
#define _COMMUNICATE_H_

#include "LogWrapper/LogWrapper.h"
#include "CommunicateDef/CommunicateDef.h"

namespace algo
{


#ifdef _BOOST_SERIALIZE_

template<typename Archive>
void ST_ROAD_DriveLane::serialize(Archive& ar, unsigned int version){
    ar &nStartKF &nEndKF &nLeftLP &nRightLP;
}

template<typename Archive>
void ST_ROAD_Point3D::serialize(Archive& ar, unsigned int version){
    ar &x  &y  &z;
}

template<typename Archive>
void ST_ROAD_LineParam::serialize(Archive& ar, unsigned int version){
    ar &nInputType  &nOutputType  &nVariable  &nMaxPower;
    ar &coef;
}

template<typename Archive>
void ST_ROAD_LineModel::serialize(Archive& ar, unsigned int version){
    ar &nSubItemType  &nLineWidth;
    ar &vecPtControl;
    ar &vecParam;
    ar &nStartKF;
    ar &nEndKF;
}

template<typename Archive>
void ST_ROAD_ItemModel::serialize(Archive& ar, unsigned int version){
    ar &nID &nItemType &fScale &fResolution;

#ifdef ENABLE_NURBS
    ar & vecPiecesNurbs;
#else
    ar & vecPieceSet;
#endif

}


#endif // _BOOST_SERIALIZE_


}



namespace roadDBCore
{
/**
 *******************************************************************************
 * @brief getArrayLength - get length of Array
 *
 *  <1> Parameter Description:
 *
 *  @param [In] t - object of array
 *
 *  @return length - length of array
 *
 *  <2> Detailed Description:
 *
 *  \ingroup
 *******************************************************************************
 */
template <class T>
uint32_t getArrayLength(IN const T & t)
{
    return sizeof(t)/sizeof(t[0]);
}

/**
 *******************************************************************************
 * @brief compareArray-compare two arrrays
 *
 *  <1> Parameter Description:
 *
 *  @param [In] t - object of array
 *
 *  @param [In] t - object of array
 *
 *  @return true-if two arrays are equal
 *
 *  <2> Detailed Description:
 *
 *  \ingroup
 *******************************************************************************
 */
template <class T>
bool compareArray(IN const T & lhs,IN const T & rhs)
{
    uint32_t rhs_size = getArrayLength(rhs);
    if (getArrayLength(lhs) != rhs_size)
    {
        return false;
    }
    for (uint32_t i=0;i!=rhs_size;++i)
    {
        if(lhs[i] != rhs[i])
        {
            return false;
        }
    }

    return true;
}

}

#endif

