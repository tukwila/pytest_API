/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IAlgoDataCollector.h
 * @brief  interface of algorithm output collector
 *******************************************************************************
 */

#ifndef IALGO_DATA_COLLECTOR_H
#define IALGO_DATA_COLLECTOR_H

#include "typeDef.h"
#include "CommunicateDef/RdbV2SSlam.h"

namespace algo {

/**
 *******************************************************************************
 * @class IAlgoDataCollector IAlgoDataCollector.h
 * @brief interface of algorithm result data collector
 *
 *
 *******************************************************************************
 */
class IAlgoDataCollector
{
public:
    typedef std::shared_ptr<roadDBCore::PayloadBase_t> AlgoData_t;

    /**
     *******************************************************************************
     * @brief get - save algorithm result data, use for algorithm
     *
     *  <1> Parameter Description:
     *
     *  @return void
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void saveAlgoData(AlgoData_t algoData) = 0;


    /**
     *******************************************************************************
     * @brief set user data, such as matrix's confidence
     *
     *  <1> Parameter Description:
     *  userData: must allocate and free by caller.
     *  @return void
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    virtual void setUserData(const void *userData) = 0;

};

}


#endif

