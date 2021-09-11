/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Object.h
 * @brief  The base class for all objects.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-03-18        Tony Xiong        Init version.
 *
 *******************************************************************************
 */

#pragma once

#include <string.h>
#include <string>
#include "VehicleAPICommon.h"
#include "LogicTypes.h"

namespace RDBVehicleAPI{
/**
 * @brief  The class definition for objects.
 */
class Object
{
public:
    /**
     * @brief Convert a string based id to uint64 based id.
     *
     * @param idString The string formatted id
     * @return the uint64 based id
     *
     * Notes:
     * This method will firstly try to convert idString
     * with strtoull(); if it fails the method will use
     * std::hash to generate an id.
     * If idString is a number, it shall not be bigger than
     * 18446744073709551615 (2^64-1).
     */
    static uint64_t convertID(const std::string& idString);
    /**
     * @brief Convert a string based id to uint64 based id.
     *
     * @param idStr The string formatted id
     * @return the uint64 based id
     *
     * Notes:
     * This method will firstly try to convert idString
     * with strtoull(); if it fails the method will use
     * std::hash to generate an id.
     * When idStr is null, return 0;
     * If idString is a number, it shall not be bigger than
     * 18446744073709551615 (2^64-1).
     */
    static uint64_t convertID(const char* idStr);
    
    /**
     * @brief Convert a uint64 based id to  string based id.
     * 
     * @param id 
     * @return objectID_t 
     */
    //static objectID_t convertID(const uint64_t id);
    /**
     * @brief Construct a new Object object
     *
     */
    Object();

    /**
     * @brief Construct a new Object object
     *
     * @param id the id of this object
     */
    Object(const objectID_t& id);

    /**
     * @brief Destroy the Object object
     *
     */
    virtual ~Object();

    /**
     * @brief Get id of this object
     *
     * @return objectID_t
     */
    objectID_t getID() const;

 protected:
    Object(const Object& obj) = default;
    Object& operator=(const Object& obj) = default;

    objectID_t id_ = "";

 private:
    static bool isDigit(const char* str);
    static uint64_t doConvertID(const char* idStr, const std::string* stringPtr = nullptr);
};

// todo move to another files
class Geometry
{
};


class Visualization : public Object
{
public:
   Visualization(const objectID_t& id):Object(id){}

};
} 

