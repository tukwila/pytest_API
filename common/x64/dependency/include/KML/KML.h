/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   KML.h
 * @brief  Header file of KML module
 *******************************************************************************
 */

#ifndef _KML_H_
#define _KML_H_

#include <string>
#include <fstream>
#include "CommunicateDef/RdbV2SGeometry.h"
#include "LogWrapper/LogWrapper.h"

namespace roadDBCore
{

enum Geometry_E
{
    Geometry_E_Point,
    Geometry_E_LineString,
    Geometry_E_Polygon,
    Geometry_E_MultiGeometry
};

enum POINT_STYLE_E
{
	POINT_STYLE_RED_SMALL_E,
	POINT_STYLE_RED_MID_E,
	POINT_STYLE_RED_BIG_E,
	POINT_STYLE_GREEN_SMALL_E,
	POINT_STYLE_GREEN_MID_E,
	POINT_STYLE_GREEN_BIG_E,
	POINT_STYLE_BLUE_SMALL_E,
	POINT_STYLE_BLUE_MID_E,
    POINT_STYLE_BLUE_BIG_E,
    POINT_STYLE_PINK_SMALL_E,
	POINT_STYLE_PINK_MID_E,
	POINT_STYLE_PINK_BIG_E
};

enum  KML_ERROR
{
     KML_OK = 0,
     KML_FILE_OPEN_FAILED = 1,
     KML_ERROR = 2
};
class KML
{
public:
    KML();
    KML(const std::string fileName);
    ~KML();

    int32_t addPlaceMarkBegin(const std::string &name = "", const std::string &description = "",const int& visiblity = 1);

    void addPointBegin();

    //================================
    void addGlobalStyle();
    void addPolygon(Point3d_t gps);
    void addPolygon(const std::vector<Point3d_t> &vGps, const std::string &name, const std::string &description, const std::string &color, const int& visiblity = 1);
    void addPolygons(const std::vector<Point3d_t> &vGps, const std::string &name, const std::string &styleID);
    int32_t addPoint(const Point3d_t &gps, const std::string &name, const std::string &desc = "", const int &visiblity = 1, const std::string &style = "s_ylw-pushpin");
    //================================

    int32_t addColorPoint(const Point3d_t &gps, const std::string &name="");

    int32_t addPoint(const Point3d_t &gps, float width = 0.2f, const std::string &color = "ff0000ff", const std::string &name="", const std::string &desc="",const int& visiblity = 1);
	int32_t addPointWithStyle(const Point3d_t &gps, const std::string &name="", const POINT_STYLE_E &style=POINT_STYLE_RED_SMALL_E, const std::string &desc="", const int& visiblity=1);
    void    addPoints(const std::vector<Point3d_t> &vGps, const int& visiblity = 1);
    void    addPointsWithID(const std::vector<std::pair<Point3d_t, uint64_t>> &vGpsWithID, const int& visiblity = 1, const bool bDebug = true);
    int32_t addLineString(const std::vector<Point3d_t> &vGps, const int& width = 2.0, const std::string &normalColor = "ff00ffff", const std::string &lineName=""); //format of color:0xaarrggbb
    int32_t addLineString(const std::vector<Point3d_t> & vGps, std::string drawOrder);
    int32_t addMultiGeometryBegin();
    int32_t addMultiGeometryEnd();
    int32_t    addPlaceMarkEnd();
    void    addPlaceMarkStyle(const std::string &normalColor = "ffffd041", const float &scale = 1.0);
    void    addPlaceMark(const Point3d_t &gps, const std::string &name);
    void    addPlaceMark(const Point3d_t &gps, const std::string &name, const std::string & styleID, int iVisibility, const std::string & desc = "");
    void    addPlaceMarkWithStyle(const Point3d_t &gps, const std::string &name, const std::string & styleID, const std::string & desc = "");
    int32_t addPointColor();
    int32_t addFolderBegin(const std::string &name = "", bool open = false, const std::string &description = "");
    int32_t addFolderEnd();
    int32_t addNameOpen(const std::string &name);
    int32_t addDocumentHead();
    int32_t addDocumentHead(const std::string & description);
    int32_t addDocumentEnd();
    int32_t addTypeNameLine(const std::string &typeName);
    void    addMarkPin(const Point3d_t &gps, const std::string & segmntIDStr,const int& visibility=1);
    int32_t addVisibility(int iVisibility);
    int32_t addArrowMarkBegin(const std::string &arrowName);
    int32_t addArrowMarkEnd(const std::string &color);
    int32_t addCoordinates(const std::vector<Point3d_t> &vGps);

    void dumpPoints2Kml(const std::string            &filename,
                        const std::vector<Point3d_t> &points,
                        const std::string            &color= "FF00FFFF",
                        const bool                    bLineMode=true);

    void dumpPlacemark(const Point3d_t   &point,
                       const std::string &filename,
                       const std::string &color = "FF00FFFF");
    int32_t dumpArrowToKml(
            const roadDBCore::Point3d_t &refGps,
            const roadDBCore::Point3d_t &currCenterPt,
            const roadDBCore::Point3d_t &relatedCenterPt,
            const std::string &arrowName,
            const std::string &color,
            const int32_t precision,
            const int32_t &visiblity = 1);
    static const char *getColor(std::size_t idx);

	static const std::string getPointStyleID(const POINT_STYLE_E &style);

private:
    void addHeader();
    void addTail();

private:
    std::ofstream ofile_;
    std::string lineColor_;
    std::string lineName_;

    float width_;
};

}

#endif
