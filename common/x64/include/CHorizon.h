/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   LandScape.h
 * @brief  Definition of LandScape and sub classes
 *******************************************************************************
 */
 
#pragma once

#include "Landscape.h"
#include "Junction.h"
#include  "VehicleAPICommon.h"

#include <memory>

namespace RDBVehicleAPI
{
#define WP_CONST(T)     std::weak_ptr<const T> 
#define WPVec(T) 		  const std::vector<std::weak_ptr<const T>>&

#define SP_CONST(T)     std::shared_ptr<const T>
#define SPVec(T) 		  const std::vector<std::shared_ptr<const T>>&
class CHorizon : public std::enable_shared_from_this<CHorizon>
{
	friend class ComnApiImp;
	
public:
	CHorizon();
	virtual ~CHorizon();
	
public:
	void addJunction(const SP_CONST(Junction)& pJunctions);
	void addLandscape(const SP_CONST(Landscape)& pLandscapes);
	
    const std::vector<std::shared_ptr<const Landscape>>& getLandscapes() const ;
    const std::vector<std::shared_ptr<const Junction>>& getJunctions() const;

private:
    void reset();
private:
    std::vector<std::shared_ptr<const Landscape>> pLandscapes_;
    std::vector<std::shared_ptr<const Junction>> pJunctions_;
};


}

