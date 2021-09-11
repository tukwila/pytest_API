/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RoadGeometryImpl.h
 * @brief  interface of roadGeometry 
 *******************************************************************************
 */

#ifndef ROAD_GEOMETRY_IMPL_H
#define ROAD_GEOMETRY_IMPL_H

#include "typeDef.h"

#include "algoInterface/IRoadGeometry.h"

namespace roadDBCore {

class RoadGeometryPayload_t;


/**
 *******************************************************************************
 * @class RoadGeometryImpl RoadGeometryImpl.h
 * @brief interface of road geometry objects
 *
 *
 *******************************************************************************
 */
class RoadGeometryImpl : public IRoadGeometry
{
public:

    virtual ~RoadGeometryImpl(){};
    /**
      *******************************************************************************
      * @brief putGeometry/getGeometry - put/get road geometry
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
    virtual void putGeometry(const std::shared_ptr<RoadGeometryPayload_t> &road) ; 

	virtual  const std::shared_ptr<RoadGeometryPayload_t> & getGeometry() const;
	
private:
	std::shared_ptr<RoadGeometryPayload_t>  roadGeometry_;
    
};

}


#endif

