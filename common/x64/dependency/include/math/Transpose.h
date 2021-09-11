/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Transpose.h
 * @brief  Define the implementation of Matrix Transpose
 *******************************************************************************
 */

#ifndef TRANSPOSE_H
#define TRANSPOSE_H


#include "../thirdParty/eigen/Eigen/Core"
#include "../thirdParty/eigen/Eigen/Dense"

#include "Base.h"

using namespace std;

namespace roadDBCore {

namespace internal {

template<typename XprType>
struct traits< Transpose<XprType> >
{
  typedef typename XprType::Scalar Scalar;

};

}


template<typename Derived>
class MatBase_t;


template<typename XprType>
class Transpose: public roadDBCore :: MatBase_t< Transpose<XprType> >
{
public:

    typedef typename roadDBCore :: MatBase_t< Transpose<XprType> > Base;
    typedef typename internal::traits<XprType>::Scalar Scalar;
    inline Transpose(XprType& xpr)
          :m_xpr(xpr)
    {
        mat = m_xpr.matEigen_.transpose();
        //cout << "mat = \n" << mat << endl;
    }

    ~Transpose(){};


    const Scalar coeff(int row, int col) const
    {

        //cout << "Enter Block::coeff() \n" << endl;


        return mat(row , col);
    }

protected:

    XprType& m_xpr;
    Eigen::MatrixXd mat;
};





}

#endif
