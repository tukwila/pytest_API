/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Paint.h
 * @brief  The class definition of Paint.
 *
 * Change Log:
 * Date              Author            Changes
 * 2020-06-18        Wei Tian          Init version.
 *
 *******************************************************************************
 */

#pragma once

#include <vector>
#include <utility>
#include <memory>
#include "VehicleAPICommon.h"
#include "Line.h"
#include "LogicTypes.h"

#include <memory>

namespace RDBVehicleAPI
{
  class Line;

  class Paint : public Visualization, public std::enable_shared_from_this<Paint>
  {
  public:
    ~Paint();

    /** 
     * @brief Get the line object expressed this paint.
     * @return line object.   
     */
    std::shared_ptr<const Line> &getExpressedLine()
    {
      return expressedLine_;
    }

  private:
    void setExpressedLine(const std::shared_ptr<const Line> &line)
    {
      expressedLine_ = line;
    }

    objectID_t getExpressedLineID() { return expressedLineId_; }

    Paint(const objectID_t &id, const objectID_t &expressedLineId);
    Paint() = delete;
    Paint(const Paint &obj) = delete;
    Paint &operator=(const Paint &obj) = delete;

  private:
    std::shared_ptr<const Line> expressedLine_;
    objectID_t expressedLineId_;

    FRIEND_2_ROADDATAINFO;
  };
} /* namespace RDBVehicleAPI */
