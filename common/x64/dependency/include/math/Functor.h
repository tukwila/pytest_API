/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Functor.h
 * @brief  Define the implementation of MatExpress Functor type
 *******************************************************************************
 */
#ifndef FUNCTOR_H
#define FUNCTOR_H

namespace roadDBCore {

namespace internal {

    template<typename Scalar> struct scalar_plus {
        inline const Scalar operator() (const Scalar& a, const Scalar& b) const
        {
            return a + b;
        }
    };

    template<typename Scalar> struct scalar_sub {
        inline const Scalar operator() (const Scalar& a, const Scalar& b) const
        {
            return a - b;
        }
    };

}//internal end

}//roadDBCore end

#endif
