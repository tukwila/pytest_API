/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ITrafficSign.h
 * @brief  interface of traffic sign
 *******************************************************************************
 */
#ifndef ITRAFFIC_SIGN_H_
#define ITRAFFIC_SIGN_H_

#include <vector>
#include <opencv2/imgproc.hpp>
#include "typeDef.h"
#include "CommunicateDef/Communicate.h"
#include "CommunicateDef/RdbV2SRoadObject.h"

namespace algo
{
/**
 *******************************************************************************
 * @class ITrafficSignIn ITrafficSign.h
 * @brief interface of TrafficSingIn
 *
 *  get the traffic sign from the interface
 *******************************************************************************
 */

struct TrafficSign
{
   int32_t           type;
   /* the orientation of the sign,
    * 0 means face East, PI/2 means face North,
    * PI means face West, 3PI/2 means face South*/
   float32_t         orientation;
   float32_t         shapeWidth; //the width of traffic sign, in meter
   float32_t         shapeHeight; //the height of traffic sign, in meter
   float32_t         confidence; //0-1  the reliabity is higher when the value is larger
   /* the relative position in meter to the location of the first frame,
    * x axis towards North,
    * y axis towards East,
    * z axis toward up */
   cv::Point3f       pos;
   std::vector<ROI>  vRoi;
   int32_t startKFIdx;
   int32_t endKFIdx;
};

class ITrafficSignIn
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
    virtual bool get(IN int32_t index, OUT std::shared_ptr<roadDBCore::RoadObject_t> &sign) const = 0;
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
 * @class ITrafficSignOut ITrafficSign.h
 * @brief interface of ITrafficSignOut
 *
 * put the result to the interface
 *******************************************************************************
 */

class ITrafficSignOut
{
public:
   /**
      *******************************************************************************
      * @brief get - put the traffic sign to the interface
      *
      *  <1> Parameter Description:
      *
      *  @param [In]  - sign   the traffic sign to put the interface
      *
      *  @return success return true, or return false
      *
      *  <2> Detailed Description:
      *
      *  \ingroup algorithmAdapt
      *******************************************************************************
      */
   virtual bool put(IN std::shared_ptr<roadDBCore::RoadObject_t> sign) = 0;
};
}
#endif
