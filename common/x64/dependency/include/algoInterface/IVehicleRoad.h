/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IVehicleRoad.h
 * @brief  file of 3D lane modle input and output interface file
 *******************************************************************************
 */
#ifndef IVEHICLE_ROADOUT_H_
#define IVEHICLE_ROADOUT_H_

#include "algoInterface/RoadModelDefs.h"

namespace algo
{
class IVehicleRoadOut
{
public:
      /**
      *******************************************************************************
      * @brief put - put road model from vehicle side
      *
      *  <1> Parameter Description:
      *
      *  @param [Out]  - roadmodels   road items represented by model
      *
      *  @return success return true, or return false
      *
      *  <2> Detailed Description:
      *
      *
      *  \ingroup algorithm
      *******************************************************************************
      */
      virtual bool put(IN XhItemModelSet& roadmodels) = 0;


      /**
      *******************************************************************************
      * @brief put - put road model from vehicle side
      *
      *  <1> Parameter Description:
      *
      *  @param [Out]  - roadgeometry   road items and lanes
      *
      *  @return success return true, or return false
      *
      *  <2> Detailed Description:
      *
      *
      *  \ingroup algorithm
      *******************************************************************************
      */
      virtual bool put(IN ST_ROAD_Geometry& roadgeometry) = 0;
};

class IVehicleRoadIn
{
public:
     /**
      *******************************************************************************
      * @brief get - get road model from vehicle side
      *
      *  <1> Parameter Description:
      *
      *  @param [Out]  - roadmodels   road items represented by model
      *
      *  @return success return true, or return false
      *
      *  <2> Detailed Description:
      *
      *
      *  \ingroup implement
      *******************************************************************************
      */
      virtual bool get(OUT XhItemModelSet& roadmodels) = 0;


      /**
       *******************************************************************************
       * @brief get - get road model from vehicle side
       *
       *  <1> Parameter Description:
       *
       *  @param [Out]  - roadgeometry   road items and lanes
       *
       *  @return success return true, or return false
       *
       *  <2> Detailed Description:
       *
       *
       *  \ingroup implement
       *******************************************************************************
       */
      virtual bool get(OUT ST_ROAD_Geometry& roadgeometry) = 0;
};
}
#endif
