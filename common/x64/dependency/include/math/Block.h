/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Block.h
 * @brief  Define the implementation of Block
 *******************************************************************************
 */

#ifndef BLOCK_H
#define BLOCK_H


#include "../thirdParty/eigen/Eigen/Core"


using namespace std;

namespace roadDBCore {


namespace internal {

template<typename XprType>
struct traits< Block<XprType> >
{
  typedef typename XprType::Scalar Scalar;

};

}


template<typename Derived>
class MatBase_t;



template<typename XprType>
class Block: public roadDBCore :: MatBase_t< Block<XprType> >
{
public:

    typedef typename roadDBCore :: MatBase_t< Block<XprType> > Base;
    typedef typename internal::traits<XprType>::Scalar Scalar;

    inline Block(XprType& xpr,
          int a_startRow, int a_startCol,
          int blockRows, int blockCols)
          :m_xpr(xpr)
    {
        //cout << "Enter Block::Block() \n" << endl;
        //cout << "after m_xpr = xpr; \n" << endl;
        m_startRow = a_startRow;
        m_startCol = a_startCol;
        m_blockRows = blockRows;
        m_blockCols = blockCols;
        mat = m_xpr.matEigen_.block(m_startRow , m_startCol , m_blockRows , m_blockCols);

        //cout << "mat = \n" << mat << endl;
        //cout << "m_xpr.matEigen_ = \n" << m_xpr.matEigen_ << endl;

        //cout << "Enter Block::Block() \n" << endl;
    }

    ~Block(){};


    const Scalar coeff(int row, int col) const
    {

        //cout << "Enter Block::coeff() \n" << endl;


        return mat(row , col);
    }

protected:

    XprType& m_xpr;
    int m_startRow;
    int m_startCol;
    int m_blockRows;
    int m_blockCols;
    Eigen::MatrixXd mat;



};





}

#endif
