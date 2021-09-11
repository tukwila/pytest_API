/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   MatSVD.h
 * @brief  Define the implementation of Matrix SVD decomposition
 *******************************************************************************
 */

#ifndef MATSVD_H
#define MATSVD_H


#include "../thirdParty/eigen/Eigen/Core"
#include "../thirdParty/eigen/Eigen/Dense"
#include "Mat_t.h"


using namespace std;
using namespace Eigen;

namespace roadDBCore {

template<typename _MatrixType>
class MatSVD
{
public:

    MatSVD(const _MatrixType& matrix)
    {
        row = _MatrixType::rowstest;
        col = _MatrixType::colstest;
        MatrixXd m = matrix.matEigen_;
        JacobiSVD< Matrix<typename _MatrixType::Scalar, _MatrixType::rowstest, _MatrixType::colstest> > svd(m , ComputeFullU | ComputeFullV);
        m_matU.matEigen_ = svd.matrixU();
        m_matV.matEigen_ = svd.matrixV();

        for(int i = 0 ; i < row*col ; i ++)
        {
            m_singularValues(i) = 0;
        }

        int min = (row < col)? row : col;
        MatrixXd temp;
        temp = svd.singularValues();
        for(int i = 0 ; i < min ; i++)
        {
            m_singularValues(i,i) = temp(i);
        }

    }

    ~MatSVD(){};

    const Mat_t<typename _MatrixType::Scalar,_MatrixType::rowstest,_MatrixType::rowstest> matrixU() const
    {
        return m_matU;
    }

    const Mat_t<typename _MatrixType::Scalar,_MatrixType::colstest,_MatrixType::colstest> matrixV() const
    {
        return m_matV;
    }

    const Mat_t<typename _MatrixType::Scalar,_MatrixType::rowstest,_MatrixType::colstest> singularValues() const
    {
        return m_singularValues;
    }



protected:
    int row;
    int col;
    Mat_t<typename _MatrixType::Scalar,_MatrixType::rowstest,_MatrixType::rowstest> m_matU;
    Mat_t<typename _MatrixType::Scalar,_MatrixType::colstest,_MatrixType::colstest> m_matV;
    Mat_t<typename _MatrixType::Scalar,_MatrixType::rowstest,_MatrixType::colstest> m_singularValues;

};





}

#endif
