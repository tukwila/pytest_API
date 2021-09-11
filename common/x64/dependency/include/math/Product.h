/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Product.h
 * @brief  Define the implementation of Mat Product
 *******************************************************************************
 */

#ifndef PRODUCT_H
#define PRODUCT_H


#include "../thirdParty/eigen/Eigen/Core"


using namespace std;

namespace roadDBCore {


namespace internal {

template<typename Lhs, typename Rhs>
struct traits< Product<Lhs,Rhs> >
{
  typedef typename Lhs::Scalar Scalar;

};

}


template<typename Derived>
class MatBase_t;

template<typename Lhs, typename Rhs>
class ProductImpl;

template<typename Lhs, typename Rhs>
class Product: public ProductImpl<Lhs , Rhs>
{
public:

    typedef ProductImpl<Lhs, Rhs> Base;
    typedef typename internal::traits< Product<Lhs,Rhs> >::Scalar Scalar;

    Product(const Lhs& mLhs, const Rhs& mRhs)
    : m_lhs(mLhs), m_rhs(mRhs)
    {


    }

    ~Product(){};

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

};



template< typename Lhs, typename Rhs >
class ProductImpl
: public roadDBCore :: MatBase_t< Product<Lhs , Rhs> >
{
    typedef  Product< Lhs , Rhs> Derived;
    public:
    typedef typename roadDBCore :: MatBase_t< Product<Lhs , Rhs> > Base;
    typedef typename internal::traits<Derived>::Scalar Scalar;


    const Scalar coeff(int row , int col) const
    {
        return multiply(this->derived().lhs() , this->derived().rhs() , row , col);
    }


    template<typename T1 , typename T2>
    const Scalar multiply(const T1& LMat , const T2& RMat , int row , int col) const
    {
        int size = T1::rowstest * T2::colstest;
        typename T1::Scalar temp  = 0;
        int Mulrows = T1::rowstest;
        int Mulcols = T2::colstest;
        int innerSize = T1::colstest;
        int indexRow = 0;
        int indexCol = 0;


            indexRow = row;
            indexCol = col;
            for(int i = 0; i < innerSize ; i++)
            {
                temp += LMat.matEigen_.coeff(indexRow,i) * RMat.getVal(i,indexCol);
                //cout << "temp = " << temp << endl;
            }
            //cout << "indexRow = " << indexRow << endl;
            //cout << "indexCol = " << indexCol << endl;

        return temp;
    }


};



}//end namespace roadDBCore
#endif
