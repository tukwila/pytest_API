/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   LightKML.h
 * @brief  Header file of LightKML module.
 *         LightKML is a light, high performance kml file generator.
 *******************************************************************************
 */

#ifndef _LIGHTKML_H_
#define _LIGHTKML_H_

#include <string>
#include <fstream>
#include "CommunicateDef/RdbV2SGeometry.h"
#include "LogWrapper/LogWrapper.h"

namespace roadDBCore
{

class BEHandler;
typedef std::shared_ptr<BEHandler> BEHPtr;

class LightKML
{
public:
    LightKML(const std::string &fileName, const std::string &exeDir = "");
    ~LightKML();

    BEHPtr addHeader();

    BEHPtr addDocStyle(const std::string &name = "", const std::string &desc = "", bool embedColor360 = false, int width = 3);

    BEHPtr addFolder(const std::string &name = "", int open = 0, const std::string &desc = "");

    BEHPtr addPlaceMark(const std::string &style, const std::string &name = "",
                        int visibility = 1, const std::string &desc = "", const bool bColor = false);

    BEHPtr addMultiGeometry();
    void addPolygon(const Point3d_t &gps);
    void addPolygons(const std::vector<Point3d_t> &vGps, const std::string &name,
                     const std::string &styleID, int visibility = 1, bool isShowAlt = false);

    void addFakePoint(const Point3d_t &pt);
    void addFakePoints(const std::vector<Point3d_t> &vGps, const std::string &name,
                       const std::string &styleID, bool isShowAlt = false);

    BEHPtr addPoint();

    BEHPtr addLineString(const std::string &altitudeMode = "");

    BEHPtr addCoordinate();
    void addCoordinateContent(const Point3d_t &pt);

    void addCoordinate(const Point3d_t &pt);
    void addCoordinate(const Point3f_t &pt);
    void addCoordinates(const std::vector<Point3d_t> &vPt);
    void addCoordinates(const std::vector<Point3f_t> &vPt);

    void addArrow(const std::vector<Point3d_t> &vecGps, const std::string &style, const std::string &name);
    void addNetworkLink(const std::string &name, const std::string &href, int visibility = 1);

    LightKML(const LightKML &) = delete;
    LightKML & operator=(const LightKML &) = delete;

public:
    static const int resPathNum = 6;
    static const char* resourcePaths[resPathNum];
    static const char* styleFileName;
    static const char* color360FileName;
    static const char* dotFileName;

    static std::string getExeParentPath();

private:
    std::string outputFileName;
    std::string dotFilePath;
    std::string styleFilePath;
    std::string color360FilePath;

    std::ofstream ofs;
    void addHeaderEnd();
    void addDocStyleEnd();
    void addFolderEnd();
    void addPlaceMarkEnd();
    void addMultiGeometryEnd();
    void addPointEnd();
    void addLineStringEnd();
    void addCoordinateEnd();

    bool searchFile(std::string &accessedFullPath, const std::string &path, const char * fileName);
    void generateColor360Style(std::string &color360Style, int width = 3);
};


class BEHandler
{
public:
    typedef void (LightKML::*HANDLER)();

    BEHandler(LightKML * pKML, HANDLER pHandler)
    {
        //LOG_INFO << "BEHandler constructor";
        this->pKml = pKML;
        this->handler = pHandler;
    }
    ~BEHandler()
    {
        //LOG_INFO << "BEHandler destructor";
        if (pKml && handler)
        {
            (pKml->*handler)();
        }
    }

    BEHandler & operator=(const BEHandler &) = delete;

private:
    HANDLER handler;
    LightKML * pKml;
};

}

#endif

