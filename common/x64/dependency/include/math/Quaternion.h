/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Quaternion.h
 * @brief  Define the implementation of Matrix Quaternion
 *******************************************************************************
 */

#ifndef QUATERNION_H
#define QUATERNION_H


#include "../thirdParty/eigen/Eigen/Core"
#include "../thirdParty/eigen/Eigen/Dense"


using namespace std;

namespace roadDBCore {


template<typename Derived>
class MatBase_t;

template<typename _Scalar>
class Quaternion
{
public:
    typedef _Scalar Scalar;

    template<typename Derived>
    explicit inline Quaternion(const MatBase_t<Derived>& other)
    {
        Eigen::Quaterniond q(other.derived().matEigen_);
        v1 = q.x();
        v2 = q.y();
        v3 = q.z();
        v4 = q.w();
    }


    inline Scalar& x() { return v1; }

    inline Scalar& y() { return v2; }

    inline Scalar& z() { return v3; }

    inline Scalar& w() { return v4; }

private:
    Scalar v1;
    Scalar v2;
    Scalar v3;
    Scalar v4;
};


typedef Quaternion<float> QuaternionfDB;

typedef Quaternion<double> QuaterniondDB;

}

#endif
