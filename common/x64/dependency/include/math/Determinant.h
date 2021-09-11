/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Determinant.h
 * @brief  Define the implementation of Matrix Determinant
 *******************************************************************************
 */

#ifndef DETERMINANT_H
#define DETERMINANT_H


#include "../thirdParty/eigen/Eigen/Core"
#include "../thirdParty/eigen/Eigen/Dense"

#include "Base.h"

using namespace std;

namespace roadDBCore {


template<typename Derived>
inline typename internal::traits<Derived>::Scalar MatBase_t<Derived>::determinant() const
{
    typename internal::traits<Derived>::Scalar detValue = 0;
    detValue = this->derived().matEigen_.determinant();
    return detValue;
}


}

#endif
