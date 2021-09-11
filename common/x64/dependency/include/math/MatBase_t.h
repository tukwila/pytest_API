/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   MatBase_t.h
 * @brief  Define the implementation of MatBase_t
 *******************************************************************************
 */

#ifndef MATBASE_T_H
#define MATBASE_T_H


#include "../thirdParty/eigen/Eigen/Core"

#include "Block.h"
#include "MatExpress.h"
#include "Product.h"
#include "Base.h"
#include "Inverse.h"
#include "Transpose.h"


using namespace std;


namespace roadDBCore {


template<typename Derived>
class MatBase_t
: public Base< Derived >
{
public:

    //typedef typename Derived::Scalar Scalar;//subclass need this type, the reason is that derived is not specify.
    typedef typename internal::traits<Derived>::Scalar Scalar;

    MatBase_t(){};
    ~MatBase_t(){};
    Scalar determinant() const;


public:

    template<typename OtherDerived>
    Derived& _set(const MatBase_t<OtherDerived>& other)
    {
        int size = this->derived().rows() * this->derived().cols();
        int rowSize = this->derived().rows();
        int colSize = this->derived().cols();

        for (int i = 0; i < rowSize; i++)
        {
            for (int j = 0; j < colSize; j++)
            {
                this->derived().matEigen_.coeffRef(i,j) = other.derived().coeff(i,j);
                //cout << "Enter _set() and i = " << i << endl;
                //cout << "Enter _set() and j = " << j << endl;
            }
        }


        return this->derived();//modify warning:warning: there are no arguments to ‘derived’ that depend on a template parameter, so a declaration of 'derived' must be available.
    }


    inline Block<Derived>
    block(int startRow, int startCol, int blockRows, int blockCols)
    {

        //cout << "Enter block in MatBase \n" << endl;

        return Block<Derived>(this->derived(), startRow, startCol, blockRows, blockCols);

    }

    inline Inverse<Derived> inverse()
    {

        //cout << "Enter inverse in MatBase \n" << endl;

        return Inverse<Derived>(this->derived());

    }


    inline Transpose<Derived> transpose()
    {

        //cout << "Enter inverse in MatBase \n" << endl;

        return Transpose<Derived>(this->derived());

    }
    //operator+
    template<typename OtherDerived>
    const MatExpress<internal::scalar_plus< Scalar > , const Derived, const OtherDerived>
    operator+(const MatBase_t<OtherDerived>& other) const
    {

        //cout << "Enter operator+ \n" << endl;

        return MatExpress<internal::scalar_plus< Scalar > , const Derived, const OtherDerived>(this->derived(),other.derived());
    }

    //operator-
    template<typename OtherDerived>
    const MatExpress<internal::scalar_sub< Scalar > , const Derived, const OtherDerived>
    operator-(const MatBase_t<OtherDerived>& other) const
    {

        //cout << "Enter operator- \n" << endl;

        return MatExpress<internal::scalar_sub< Scalar > , const Derived, const OtherDerived>(this->derived(),other.derived());
    }


    //operator*
    template<typename OtherDerived>
    const Product<const Derived, const OtherDerived>
    operator*(const MatBase_t<OtherDerived>& other) const
    {

        //cout << "Enter operator* \n" << endl;

        return Product<const Derived, const OtherDerived>(this->derived(),other.derived());
    }





private:


};

}
#include "Determinant.h"
#endif
