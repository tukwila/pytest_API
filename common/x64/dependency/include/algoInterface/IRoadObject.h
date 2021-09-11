/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IRoadObject.h
 * @brief  interface of road object
 *******************************************************************************
 */
#ifndef IROAD_OBJEDT_H_
#define IROAD_OBJEDT_H_

#include <vector>
#include <opencv2/imgproc.hpp>
#include "typeDef.h"
#include "CommunicateDef/Communicate.h"
#include "CommunicateDef/RdbV2SRoadObject.h"

namespace algo
{

class IRoadObjectIn
{
public:
    /**
      *******************************************************************************
      * @brief get - get the result of traffic sign
      *
      *  <1> Parameter Description:
      *  @param [In]  - index  from 0
      *
      *  @param [Out]  - sign  the traffic sign
      *  @return true if get the traffic sign, or false
      *
      *  <2> Detailed Description:
      *
      *  \ingroup algorithmAdapt
      *******************************************************************************
      */
    virtual bool get(IN int32_t index, OUT std::shared_ptr<roadDBCore::RoadObject_t> &obj) const = 0;

    /**
      *******************************************************************************
      * @brief getNum - get the number of all traffic signs
      *
      *  <1> Parameter Description:
      *
      *  @return the number of traffic signs
      *
      *  <2> Detailed Description:
      *
      *  \ingroup algorithmAdapt
      *******************************************************************************
      */
    virtual int32_t getNum() const = 0;

    virtual roadDBCore::RoadGeometryPayload_t &getPayload() = 0;

};

/**
 *******************************************************************************
 * @class IRoadObjectOut
 * @brief interface of Road Object
 *
 * put the result to the interface
 *******************************************************************************
 */

class IRoadObjectOut
{
public:
   /**
      *******************************************************************************
      * @brief get - put the road object to the interface
      *
      *  <1> Parameter Description:
      *
      *  @param [In]  - sign   the road object to put the interface
      *
      *  @return success return true, or return false
      *
      *  <2> Detailed Description:
      *
      *  \ingroup algorithmAdapt
      *******************************************************************************
      */
   virtual bool put(IN std::shared_ptr<roadDBCore::RoadObject_t> obj) = 0;
};
}
#endif
