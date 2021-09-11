/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Base.h
 * @brief  Define the implementation of Base
 *******************************************************************************
 */

#ifndef BASE_H
#define BASE_H



using namespace std;

namespace roadDBCore {


template<typename Derived> struct Base
{
    Derived& derived()
    {
        //cout << "Enter Base::derived() \n" << endl;
        return *static_cast<Derived*>(this);
    }

    const Derived& derived() const
    {
        //cout << "Enter Base::const derived \n" << endl;
        return *static_cast<const Derived*>(this);
    }

    /*
    // const_cast_ptr() is important to :m_xpr(xpr)
    template<typename T>
    inline T* const_cast_ptr(const T* ptr)
    {
        cout << "Enter Base::const_cast_ptr() \n" << endl;
        return const_cast<T*>(ptr);
    }
    */


};



}

#endif
