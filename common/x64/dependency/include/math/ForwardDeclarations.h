/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ForwardDeclarations.h
 * @brief  Define the implementation of MatBase_t
 *******************************************************************************
 */

#ifndef FORWARDDECLARATIONS_H
#define FORWARDDECLARATIONS_H

#include "Functor.h"
#include "Quaternion.h"


using namespace std;


namespace roadDBCore {

namespace internal{

    template<typename T> struct traits;
    template<typename T> struct traits<const T> : traits<T> {};

}

template<typename _Scalar, int _Rows, int _Cols> class Mat_t;
template<typename BinaryFunctor , typename Lhs, typename Rhs> class MatExpress;
template<typename Lhs, typename Rhs> class Product;
template<typename XprType> class Block;
template<typename XprType> class Inverse;
template<typename XprType> class Transpose;


}

#endif
