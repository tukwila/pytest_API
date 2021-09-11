/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   TrafficSignImp.h
 * @brief  ITrafficSign interface implementation class
 *******************************************************************************
 */

#ifndef TRAFFIC_SIGN_IMPL_H
#define TRAFFIC_SIGN_IMPL_H
#include "algoInterface/ITrafficSign.h"
#include <boost/serialization/access.hpp>
#include "algoInterface/TrafficSignArDef.h"

namespace roadDBCore
{
struct RoadGeometryPayload_t;

class TrafficSignImp : public algo::ITrafficSignIn, public algo::ITrafficSignOut
{
public:
    TrafficSignImp();
    virtual ~TrafficSignImp();
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
    virtual bool put(IN const std::shared_ptr<RoadObject_t> sign);
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
    virtual int32_t getNum() const;
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
    virtual bool get(IN roadDBCore::int32_t index, OUT std::shared_ptr<RoadObject_t> &sign) const;
    virtual RoadGeometryPayload_t &getPayload();
    
    void clear();

    void convertFromArFormat();

    void convertToArFormat();

    vecTrafficSignAr *getTsAr();

private:
    RoadGeometryPayload_t   roadGeometery;
    std::vector<std::shared_ptr<RoadObject_t>> *pVecRoadObject;
    std::vector<algo::TrafficSign> trafficSigns_;
    vecTrafficSignAr tsAr_;
    
};
}

#endif
