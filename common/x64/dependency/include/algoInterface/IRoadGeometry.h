/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IRoadGeometry.h
 * @brief  interface of roadGeometry 
 *******************************************************************************
 */

#ifndef IROAD_GEOMETRY_H
#define IROAD_GEOMETRY_H

#include "typeDef.h"
#include "CommunicateDef/RdbV2SRoadObject.h"

namespace roadDBCore {

class RoadGeometryPayload_t;

/**
 *******************************************************************************
 * @class ISlamCollector ISlamCollector.h
 * @brief interface of slam output collector
 *
 *
 *******************************************************************************
 */
class IRoadGeometry
{
public:

    virtual ~IRoadGeometry(){};
    /**
      *******************************************************************************
      * @brief putGeometry/getGeometry - put/get road geometry objects allocate memory by share_ptr
      *
      *  <1> Parameter Description:
      *
      *
      *  <2> Detailed Description:
      *
      *
      *  \ingroup algorithmAdapt
      *******************************************************************************
    */   
    virtual void putGeometry(const std::shared_ptr<RoadGeometryPayload_t> &roads) = 0; 

	virtual  const std::shared_ptr<RoadGeometryPayload_t> & getGeometry() const = 0;
    
};

}


#endif

