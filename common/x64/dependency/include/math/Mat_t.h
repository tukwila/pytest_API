/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Mat_t.h
 * @brief  Define the implementation of Mat_t
 *******************************************************************************
 */

#ifndef MAT_T_H
#define MAT_T_H

#define USE_EIGEN

#ifdef USE_EIGEN
#include "../thirdParty/eigen/Eigen/Core"
#endif

#include "MatBase_t.h"

using namespace std;


namespace roadDBCore {


namespace internal {

template<typename T, int _Rows, int _Cols>
struct traits< Mat_t<T, _Rows, _Cols> >
{
  typedef T Scalar;
  enum {
    RowsAtCompileTime = _Rows,
    ColsAtCompileTime = _Cols,
  };
};

}



/*******************************************************************************
 * @class Mat_t
 * @brief This is a the basic matrix data type
 *******************************************************************************
 */
template<typename T, int ROWS, int COLS>
class Mat_t
    : public MatBase_t< Mat_t<T, ROWS, COLS> >

{
public:

    typedef MatBase_t<Mat_t> Base;

    //typedef T Scalar;
    enum
    {
        rowstest = ROWS,
        colstest = COLS
    };

    /*******************************************************************************
     * @brief Mat_t() - Default constructor
     *  \ingroup Mat_t
     *******************************************************************************
     */
    Mat_t() {
        rows_ = ROWS;
        cols_ = COLS;
    }
    ;

#ifdef USE_EIGEN
    /*******************************************************************************
     * @brief Mat_t(const Eigen::Matrix<>) - Constructor
     *  @param [In]  - Matrix: initialize using this matrix
     *  \ingroup Mat_t
     *******************************************************************************
     */
    Mat_t(IN const Eigen::Matrix<T, ROWS, COLS> &inMatrix) : matEigen_(inMatrix) {rows_ = ROWS; cols_ = COLS;};
#endif

    /*******************************************************************************
     * @brief ~Mat_t() - Default destructor
     *  \ingroup Mat_t
     *******************************************************************************
     */
    ~Mat_t() {
    }
    ;


public:

    /*******************************************************************************
     * @brief getVal - Get element value from Mat
     *  @param [In]  - rowIdx
     *  @param [In]  - colIdx
     *  @return status - element value at [rowIdx, colIdx]
     *  \ingroup Mat_t
     *******************************************************************************
     */
    T getVal(IN int rowIdx, IN int colIdx) const //add const for error: passing ''as ‘this’ argument of ‘’ discards qualifiers. A const object will call non-const member functing is forbidden. This const means that this method will not modify the member variables in this class.
    {
#ifdef USE_EIGEN
        return matEigen_.coeff(rowIdx, colIdx);
#else

#endif
    };


    T getVal(int index) const
    {
        return matEigen_.coeff(index);
    }



    /*******************************************************************************
     * @brief operator() - a reference to the coefficient at given row and column.
     *  @param [In]  - rowIdx
     *  @param [In]  - colIdx
     *  @return status - a reference to memory
     *  \ingroup Mat_t
     *******************************************************************************
     */

    T& operator()(int row, int col) // A const object will call non-const member functing is forbidden. So this function may have some error ins some casese. (Need to Modify)
    {
        return matEigen_.coeffRef(row, col);
    }


    T& operator()(int index)
    {
        return matEigen_.coeffRef(index);
    }
    /*******************************************************************************
     * @brief setVal - set element value to Mat
     *  @param [In]  - rowIdx
     *  @param [In]  - colIdx
     *  @param [In]  - value
     *  \ingroup Mat_t
     *******************************************************************************
     */
    void setVal(IN int rowIdx, IN int colIdx, IN T value)
    {
#ifdef USE_EIGEN
        matEigen_(rowIdx, colIdx) = value;
#else

#endif
    };

    void setVal(int index, T value)
    {
        matEigen_.coeffRef(index) = value;
    }

    /*******************************************************************************
     * @brief rows() - rows of matrix
     *  @return status - rows of matrix
     *  \ingroup Mat_t
     *******************************************************************************
     */
    int rows() {
        return rows_;
    }
    ;

    /*******************************************************************************
     * @brief cols() - cols of matrix
     *  @return status - cols of matrix
     *  \ingroup Mat_t
     *******************************************************************************
     */
    int cols() {
        return cols_;
    }
    ;
    /*******************************************************************************
     * @brief Assigns matrices to each other
     * to prevent a default operator= from hiding the templated operator=
     *  \ingroup Mat_t
     *******************************************************************************
     */
    Mat_t& operator=(const Mat_t& other) {
        if (this == &other) // to solve A = A ?
            return *this;
        matEigen_ = other.matEigen_; // this.matEigen_ or matEigen_?
        return *this; //return this or return *this?
    }


    template<typename OtherDerived>
    Mat_t& operator=(const MatBase_t<OtherDerived>& other)
    {
        return Base::_set(other);
    }



// for debug
//private:

#ifdef USE_EIGEN
    Eigen::Matrix<T, ROWS, COLS> matEigen_;
#else

#endif
    int rows_;
    int cols_;
};

/*******************************************************************************
 * @brief mulMat - Performs matrix multiplication
 *  @param [In]  - input matrix (left)
 *  @param [In]  - input matrix (right)
 *  @param [OUT]  - output result
 *  \ingroup Mat_t
 *******************************************************************************
 */
template<typename T1, typename T2, typename T3>
int mulMat(IN const T1& otherL, IN const T2& otherR, OUT T3& Mul)
{
    typedef typename T1::Scalar Scalar;

    //bool sizeMatch = (Mul.rows() == otherL.rows()) && (Mul.cols() == otherR.cols()) && (otherL.cols() == otherR.rows());
    //if(!sizeMatch)
    //    return 0;
    int size = Mul.rows() * Mul.cols();
    Scalar temp = 0;
    int indexRow = 0;
    int indexCol = 0;
    int innerSize = otherL.cols();
    //cout << "innerSize = " << innerSize << endl;

    for(int index = 0; index < size; index++)
    {
        indexCol = (index%(Mul.cols()));
        if((indexCol == 0) &&(index != 0))
        {
            indexRow ++;
        }
        for(int i = 0; i < innerSize; i++)
        {
            temp += otherL.getVal(indexRow,i) * otherR.getVal(i,indexCol);
            //cout << "temp = " << temp << endl;
        }
        //cout << "indexRow = " << indexRow << endl;
        //cout << "indexCol = " << indexCol << endl;
        Mul.setVal(indexRow,indexCol,temp);
        temp = 0;
    }
    return 1;
}


/*******************************************************************************
 * @brief subMat - Performs matrix Sub
 *  @param [In]  - input matrix (left)
 *  @param [In]  - input matrix (right)
 *  @param [OUT]  - output result
 *  \ingroup Mat_t
 *******************************************************************************

 */
template<typename T5>
int subMat(const T5& otherL, const T5& otherR, T5& Sub)
{
    int size = Sub.rows() * Sub.cols();
    typename T5::Scalar temp = 0;
    for(int index = 0; index < size; index++)
    {
        //Sub.matEigen_.coeffRef(index) = otherL.matEigen_.coeff(index) - otherR.matEigen_.coeff(index);
        temp = otherL.getVal(index) - otherR.getVal(index);
        Sub.setVal(index , temp);
    }
    return 1;
}

}

#endif
