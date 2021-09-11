/**
 *******************************************************************************
 *                       RoadDB Confidential
 *       Copyright (c) Continental AG. 2016-2018, RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   matConvert.h
 * @brief  mat convert functions
 *******************************************************************************
 */

#ifndef _MAT_CONVERT_H_
#define _MAT_CONVERT_H_

//#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
//#include <vector>
#include "typeDef.h"
//#include "CommunicateDef/CommunicateDef.h"
#include "CommunicateDef/RdbV2SGeometry.h"


namespace roadDBCore
{

void transferQuaternionM2G(float32_t quaternion[4]);
// void CvMatToPBMatAr(const cv::Mat& img, PBMatAr& ar);
// void PBMatArToCvMat(const PBMatAr& ar, cv::Mat& img);

// std::vector<float> transferRTMatrix2Quaternion(const cv::Mat &M);
//Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

// actually a template with dimention as a nonetype parameter will be better.
// cv::Mat transferRotation2RtMatrix(const float64_t v[9]);
// cv::Mat transferQuaternion2RTMatrix (const float32_t v[4]);
// cv::Mat transferTranslation2RtMatrix(const float32_t v[3]);
// cv::Mat transferTranslation2RtMatrix(const float64_t v[3]);

// cv::Mat toCvMat(const Eigen::Matrix<float64_t,3,3> &m);

void Rbn2angEigen(const Eigen::Matrix<float64_t, 3, 3> &R_B2G,
                    float64_t &psi, float64_t &theta, float64_t &phi);
void Rbn2angEigen(const Eigen::Matrix3f &R_B2G,
                    float32_t &psi, float32_t &theta, float32_t &phi);
Eigen::Matrix<float64_t, 3, 3> ang2RbnEigen(
                    const float64_t psi, const float64_t theta, const float64_t phi);

// convert quaternion to eular angle
void Quaternion2angEigen(const float32_t v[9],
                    float64_t &psi, float64_t &theta, float64_t &phi);

// convert rotation to eular angle
void Rotation2angEigen(const float32_t v[4],
                    float64_t &psi, float64_t &theta, float64_t &phi);

template<typename T>
Point3_t<T> getPointFromRT(const T rotation[9], const T translation[3])
{
    Eigen::Matrix3d R_G2C = Eigen::Matrix3d::Zero();

    R_G2C << rotation[0], rotation[1], rotation[2],
             rotation[3], rotation[4], rotation[5],
             rotation[6], rotation[7], rotation[8];

    Eigen::Vector3d t_GinC;
    t_GinC << translation[0], translation[1], translation[2];

    Eigen::Vector3d ow = -R_G2C.transpose() * t_GinC;

    return Point3_t<T>(ow(0), ow(1), ow(2));
}

Point3f_t getPointFromQT(const float32_t quaternion[4], const float32_t translation[3]);

void transferRBG2NEDEuler(const Eigen::Matrix3f& R_B2G,
                          float32_t& yaw_NED, float32_t& pitch_NED, float32_t& roll_NED);
}

#endif
