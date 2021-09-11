/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Node.h
 * @brief  The class definition of Node.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-11-26        Yajun Zhao        Init version.
 *
 *******************************************************************************
 */

#pragma once
#include "VehicleAPICommon.h"
#include "Object.h"
#include "Geo.h"

namespace RDBVehicleAPI
{

class Node : public Object, public std::enable_shared_from_this<Node>
{
 public:
   /**
    * @brief Destroy the Junction object
    *
    */
   ~Node(){};

	
   /**
    * @brief get the position of the Node
    * @return the postion of the Node
    */
   const WGS84_t& getPosition() const;

 private:
   Node(const objectID_t& id, const WGS84_t& position);
   Node() = default;
   Node(const Node& rhs) = delete;
   Node& operator=(const Node& obj) = delete;

 private:
   WGS84_t position_;
   FRIEND_2_ROADDATAINFO;
};
} /* namespace RDBVehicleAPI */
