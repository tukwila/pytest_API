/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   DBLayerManager.h
 * @brief  Header file of class DBLayerManager.
 *******************************************************************************
 */

#ifndef  DB_LAYER_MANAGER_H_
#define  DB_LAYER_MANAGER_H_

#include "DBCommon/DBAbstractLayer.h"
#include <map>
#include <string>
#include <memory>       // for std::shared_ptr

namespace roadDBCore
{

class DBLayerManager
{
private:
    DBLayerManager();

public:
    static DBLayerManager           * getInstance();
    std::shared_ptr<DBAbstractLayer>  getDBLayer(const std::string& dbName);
    static void                       releaseInstance();

private:
    static DBLayerManager *pManager_;
    static std::map<std::string, std::shared_ptr<DBAbstractLayer>> *pMapDbLaysers_;
};



}

#endif

