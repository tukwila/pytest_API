/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Inverse.h
 * @brief  Define the implementation of Matrix Inverse
 *******************************************************************************
 */

#ifndef INVERSE_H
#define INVERSE_H


#include "../thirdParty/eigen/Eigen/Core"
#include "../thirdParty/eigen/Eigen/Dense"

#include "Base.h"

using namespace std;

namespace roadDBCore {

namespace internal {

template<typename XprType>
struct traits< Inverse<XprType> >
{
  typedef typename XprType::Scalar Scalar;

};

}


template<typename Derived>
class MatBase_t;


template<typename XprType>
class Inverse: public roadDBCore :: MatBase_t< Inverse<XprType> >
{
public:

    typedef typename roadDBCore :: MatBase_t< Inverse<XprType> > Base;
    typedef typename internal::traits<XprType>::Scalar Scalar;
    inline Inverse(XprType& xpr)
          :m_xpr(xpr)
    {
        mat = m_xpr.matEigen_.inverse();

        //cout << "Enter Inverse::Inverse() \n" << endl;
        //cout << "m_xpr.matEigen_ = \n" << m_xpr.matEigen_ << endl;
        //cout << "mat = \n" << mat << endl;
    }

    ~Inverse(){};


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
