/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RoadEdge.h
 * @brief  The class definition of RoadEdge.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-11-26        Yajun Zhao        Init version.
 *
 *******************************************************************************
 */

#pragma once
#include "VehicleAPICommon.h"
#include "Foundation.h"
#include "Object.h"
#include "Geo.h"
#include "Line.h"
#include "LogicTypes.h"

namespace RDBVehicleAPI
{
class Line;
enum EDGE_SIDE_E
{
    EDGE_SIDE_LEFT_E       = 1,
    EDGE_SIDE_RIGHT_E      = 2
} ;

class RoadEdge : public Foundation, public std::enable_shared_from_this<RoadEdge>
{
 public:

   RoadEdgeType getType() const;
   std::shared_ptr<const Line> getLine() const;

 private:
 	// RoadEdge(const RoadEdge& other) : Foundation(other.getID(), other.getID()),pLine_(other.pLine_), side_(other.side_), type_(other.type_),height_(other.height_)
	// {
	// 	// LOG_DEBUG << "copy RoadEdge(), count=" << s_count_ ; 
	// }
  RoadEdge(const objectID_t& id, const std::shared_ptr<Line>& line, const EDGE_SIDE_E side, const RoadEdgeType type, const double height);

	
 private:
    std::shared_ptr<const Line>    pLine_;
    EDGE_SIDE_E         side_ = EDGE_SIDE_LEFT_E;
    RoadEdgeType              type_ = E_ROADEDGETYPE_MAX;       /*used to check realy type*/
    double              height_ = 0.0;
    FRIEND_2_ROADDATAINFO;
};
} /* namespace RDBVehicleAPI */
