/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   MatExpress.h
 * @brief  Define the implementation of MatExpress
 *******************************************************************************
 */

#ifndef MATEXPRESS_H
#define MATEXPRESS_H


#include "../thirdParty/eigen/Eigen/Core"


using namespace std;

namespace roadDBCore {


namespace internal {

template<typename BinaryFunctor, typename Lhs, typename Rhs>
struct traits< MatExpress<BinaryFunctor,Lhs,Rhs> >
{
  typedef typename Lhs::Scalar Scalar;

};

}


template<typename Derived>
class MatBase_t;

template<typename BinaryFunctor, typename Lhs, typename Rhs>
class MatExpressImpl;

template<typename BinaryFunctor, typename Lhs, typename Rhs>
class MatExpress: public MatExpressImpl<BinaryFunctor, Lhs , Rhs>
{
public:

    typedef MatExpressImpl<BinaryFunctor,Lhs, Rhs> Base;
    typedef typename internal::traits< MatExpress<BinaryFunctor,Lhs,Rhs> >::Scalar Scalar;

    MatExpress(const Lhs& mLhs, const Rhs& mRhs , const BinaryFunctor& func = BinaryFunctor())
    : m_lhs(mLhs), m_rhs(mRhs), m_functor(func)
    {


    }

    ~MatExpress(){};

    const Scalar coeff(int index) const
    {
        //cout << "Enter MatExpress::coeff() \n" << endl;
        return Base::coeff(index);
    }

    const Scalar coeff(int row , int col) const
    {
        //cout << "Enter MatExpress::coeff() \n" << endl;
        return Base::coeff(row,col);
    }


protected:
    Lhs& m_lhs;
    Rhs& m_rhs;
    const BinaryFunctor m_functor;

public:
    const Lhs& lhs() const
    {
        //cout << "Enter lhs() \n" << endl;
        return m_lhs;
    }
    const Rhs& rhs() const
    {
        //cout << "Enter rhs() \n" << endl;
        return m_rhs;
    }
    const BinaryFunctor& functor() const
    {
        return m_functor;
    }

};


template<typename Derived>
struct RoadBase
{
    typedef MatBase_t<Derived> type;
};

template<typename BinaryFunctor , typename Lhs, typename Rhs >
class MatExpressImpl
: public roadDBCore :: RoadBase< MatExpress<BinaryFunctor , Lhs , Rhs> >::type
{
    typedef  MatExpress<BinaryFunctor , Lhs , Rhs> Derived;
    public:
    typedef typename roadDBCore :: RoadBase< MatExpressImpl<BinaryFunctor , Lhs , Rhs> >::type Base;
    typedef typename internal::traits<Derived>::Scalar Scalar;


    const Scalar coeff(int row , int col) const
    {
        return this->derived().functor()(this->derived().lhs().matEigen_.coeff(row,col) , this->derived().rhs().matEigen_.coeff(row,col));
    }


};



}//end namespace roadDBCore
#endif
