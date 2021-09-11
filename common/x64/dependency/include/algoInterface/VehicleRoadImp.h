/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   VehicleRoadImp.h
 * @brief  vehicle road algorithm output interface implementation class
 *******************************************************************************
 */
#ifndef VEHICLE_ROAD_IMPL_H_
#define VEHICLE_ROAD_IMPL_H_

#include "algoInterface/IVehicleRoad.h"
#include "CommunicateDef/CommunicateDef.h"

namespace roadDBCore
{

class VehicleRoadImp : public algo::IVehicleRoadOut, public algo::IVehicleRoadIn
{
public:
    VehicleRoadImp();
    virtual ~VehicleRoadImp();

    /**
      *******************************************************************************
      * @brief put - put road model from vehicle side
      *
      *  <1> Parameter Description:
      *
      *  @param [IN]  - roadmodel,   road item represented by model
      *
      *  @return success return true, or return false
      *
      *  <2> Detailed Description:
      *
      *
      *  \ingroup algorithm
      *******************************************************************************
      */
    virtual bool put(IN algo::ST_ROAD_ItemModel &roadmodel);

    /**
      *******************************************************************************
      * @brief put - put road models from vehicle side
      *
      *  <1> Parameter Description:
      *
      *  @param [IN]  - roadmodels, road item represented by model
      *
      *  @return success return true, or return false
      *
      *  <2> Detailed Description:
      *
      *
      *  \ingroup algorithm
      *******************************************************************************
      */
    virtual bool put(IN algo::XhItemModelSet &roadmodels);

    virtual bool put(IN algo::ST_ROAD_Geometry &roadgeometry);

    /**
     *******************************************************************************
     * @brief getNum - get count of road models
     *
     *  <1> Parameter Description:
     *
     *  @return status - if success return number of road models , otherwise return -1.
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual int32_t getNum();

  /**
    *******************************************************************************
    * @brief get - get road model from vehicle side
    *
    *  <1> Parameter Description:
    *
    *  @param [IN]  - index
    *
    *  @param [Out]  - roadmodel   road item represented by model
    *
    *  @return success return true, or return false
    *
    *  <2> Detailed Description:
    *
    *
    *  \ingroup implement
    *******************************************************************************
    */
    virtual bool get(IN int32_t index, OUT algo::ST_ROAD_ItemModel &roadmodel);

    virtual bool get(OUT algo::ST_ROAD_Geometry& roadgeometry);

  /**
    *******************************************************************************
    * @brief get - get road model from vehicle side
    *
    *  <1> Parameter Description:
    *
    *  @param [Out]  - roadmodels   road item represented by model
    *
    *  @return success return true, or return false
    *
    *  <2> Detailed Description:
    *
    *
    *  \ingroup implement
    *******************************************************************************
    */
    virtual bool get(OUT algo::XhItemModelSet &roadmodels);

    void clear();

    void convertFromArFormat();

    void convertToArFormat();

    RoadGeometryAr_t *getRoadGeometryAr();

private:
    //algo::XhItemModelSet models_;
    algo::ST_ROAD_Geometry roadInfo_;

    RoadGeometryAr_t roadGeometryAr_;
    //vecItemModel modelsAr_;
};

}

#endif
