/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   SlamDataCollector.h
 * @brief  interface of slam output collector
 *******************************************************************************
 */

#ifndef SLAM_LOC_DATA_COLLECTOR_H
#define SLAM_LOC_DATA_COLLECTOR_H

#include "IAlgoDataCollector.h"

namespace roadDBCore {

struct RTMatrixConfidence
{
    double len;
    double avgMatchedPointNum;

    RTMatrixConfidence(): len(0.0f),avgMatchedPointNum(0.0f) {}
};

/**
 *******************************************************************************
 * @class SlamDataCollector SlamDataCollector.h
 * @brief interface of algorithm result data collector
 *
 *
 *******************************************************************************
 */
class SlamDataCollector : public algo::IAlgoDataCollector
{
public:
    typedef std::vector<algo::IAlgoDataCollector::AlgoData_t> AlgoDataSet_t;
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
    virtual void saveAlgoData(algo::IAlgoDataCollector::AlgoData_t algoData);

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
    virtual void setUserData(const void *userData);

    /**
     *******************************************************************************
     * @brief get - get algorithm result data set, use for vehicle
     *
     *  <1> Parameter Description:
     *
     *  @return AlgoDataSet_t
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    AlgoDataSet_t & getAlgoDataSet();

    /**
     *******************************************************************************
     * @brief get confidence of RTmatrix
     *
     *  <1> Parameter Description:
     *
     *  @return AlgoDataSet_t
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    RTMatrixConfidence getConfidence() const;

private:
    RTMatrixConfidence _rtConfidence;
    AlgoDataSet_t _algoDataSet;
};

}


#endif

