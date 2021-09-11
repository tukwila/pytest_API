#!/usr/bin/python3
# encoding=utf-8
#import pytest
import os, sys
import json
import copy

sys.path.append('./')
sys.path.append('..')
from config import Configs
from common import PyRIVLogicMgr

conf = Configs.config()
DBPath = conf.get('DB', 'dockerDBPath')

TestDB = {
    'mmg': {
        'proto_ver': 0.14,
        'dbPath': DBPath + 'fw-721/'
    }
}


TestDB1 = {
    'mmg': {
        'proto_ver': 0.14,
        'dbPath': DBPath + 'mmgv14_roaddbPB/'
    },
    'ck': {
        'proto_ver': 0.13,
        'dbPath': DBPath + '40db/'
    },
    'fuji': {
        'proto_ver': 0.14,
        'dbPath': DBPath + 'Fujiv14/'
    },
    'i94': {
        'proto_ver': 0.13,
        'dbPath': DBPath + 'i94/'
    }
}
TestDB_bk = {
    'WwODB': {
        'proto_ver': 0.13,
        'dbPath': '/data/Documents/loc_pb_0.13/logic_db_4.0.3_WwODB.102.rqsx.v4.pb/'
    },
    'product_pb': {
        'proto_ver': 0.13,
        'dbPath': '/data/Documents/loc_pb_0.13/20200324132012_product_pb_logicDB/'
    },
    'TEMA': {
        'proto_ver': 0.13,
        'dbPath': '/data/Documents/loc_pb_0.13/TEMA_0409_roaddbPB_data_modified/'
    }
}

curveDict = {}
lineCurveDict = {}
roadDict = {}
lanechangeDict = {}
BoundaryPaintsDict = {}


def findJSONFiles(conf_path):
    jsonFiles = []
    files = os.listdir(conf_path)
    for i in files:
        if i.endswith('.json') and not i.startswith('master') and not i.startswith('index'):
            jsonFiles.append(i)
    return jsonFiles


def getObjectsFromJson_PB14(conf_path):
    jsonFiles = findJSONFiles(conf_path)
    global curveDict
    global lineCurveDict
    global roadDict
    global lanechangeDict
    global BoundaryPaintsDict

    for i in jsonFiles:
        with open(conf_path + i, 'r') as f:
            jsonContent = json.load(f)
        # get curve of every segment
        roadList = jsonContent['landscapes']
        for r in roadList:
            # build roadDict
            roadDict[r['id']] = {}
            if 'from_landscape_ids' in r.keys():
                roadDict[r['id']]['preLandscapes'] = r['from_landscape_ids']
            else:
                roadDict[r['id']]['preLandscapes'] = []
            if 'to_landscape_ids' in r.keys():
                roadDict[r['id']]['nextLandscapes'] = r['to_landscape_ids']
            else:
                roadDict[r['id']]['nextLandscapes'] = []
            # build lineCurveDict
            for j in r['lines']:
                lineCurveDict[j['id']] = {}
                for jj in j['nurbs_curves']:
                    lineCurveDict[j['id']][jj['nurbs_express_id']] = {}
                    lineCurveDict[j['id']][jj['nurbs_express_id']]['start'] = jj['start']
                    lineCurveDict[j['id']][jj['nurbs_express_id']]['end'] = jj['end']
                    lineCurveDict[j['id']][jj['nurbs_express_id']]['length'] = jj['length']
                    for n in r['nurbs_expresses']:
                        if n['id'] in lineCurveDict[j['id']].keys():
                            lineCurveDict[j['id']][jj['nurbs_express_id']]['knots'] = n['knots']
                            lineCurveDict[j['id']][jj['nurbs_express_id']]['paint_total_length'] = n[
                                'paint_total_length']
                            lineCurveDict[j['id']][jj['nurbs_express_id']]['control_points'] = n['control_points']
                            lineCurveDict[j['id']][jj['nurbs_express_id']]['reference_point'] = jsonContent[
                                'origin_point']
            # build lanechangeDict
            for k in r['lanes']:
                evps = k['evps']
                for kk in evps:
                    if 'evp_attributes' in kk.keys():
                        lanechangeDict[kk['id']] = {}
                        evp_attributes = kk['evp_attributes']
                        lanechangeDictTemp = {}
                        for ee in evp_attributes:
                            if ee['attribute_type'] == 'EVPATTRTYPE_LANE_CHANGE':
                                lanechangeDictTemp[ee['id']] = {}
                                # getLength() is null in SDK
                                lanechangeDictTemp[ee['id']]['length'] = 0.0
                                if ee['attribute'].split(',')[0] == '0':
                                    lanechangeDictTemp[ee['id']]['isLeftChangeAllowed'] = False
                                else:
                                    lanechangeDictTemp[ee['id']]['isLeftChangeAllowed'] = True
                                    if 'left_lane_id' in k.keys():
                                        lanechangeDictTemp[ee['id']]['left_dest_lane'] = k['left_lane_id']
                                    else:
                                        lanechangeDictTemp[ee['id']]['left_dest_lane'] = None
                                if ee['attribute'].split(',')[1] == '0':
                                    lanechangeDictTemp[ee['id']]['isRightChangeAllowed'] = False
                                else:
                                    lanechangeDictTemp[ee['id']]['isRightChangeAllowed'] = True
                                    if 'right_lane_id' in k.keys():
                                        lanechangeDictTemp[ee['id']]['right_dest_lane'] = k['right_lane_id']
                                    else:
                                        lanechangeDictTemp[ee['id']]['right_dest_lane'] = None
                                lanechangeDictTemp[ee['id']]['start_position'] = ee['start_position']
                                lanechangeDictTemp[ee['id']]['end_position'] = ee['end_position']
                                lanechangeDictTemp[ee['id']]['sourceLane'] = k['id']
                                lanechangeDict[kk['id']][ee['id']] = lanechangeDictTemp[ee['id']]

        # build BoundaryPaintsDict in visualizations
        paintDict = {}
        for v in jsonContent['visualizations']:
            if v['type'] == 'VISUALIZATIONTYPE_PAINT':
                paints = v['paints']
                for vv in paints:
                    paintDict[vv['line_id']] = {}
                    paintDict[vv['line_id']]['paint_id'] = vv['id']
        for r in roadList:
            for j in r['lines']:
                if j['id'] in paintDict.keys():
                    for jj in j['nurbs_curves']:
                        paintDict[j['id']]['nurbs_express_id'] = []
                        paintDict[j['id']]['nurbs_express_id'].append(jj['nurbs_express_id'])
        paintDictNew = copy.deepcopy(paintDict)
        for r in roadList:
            laneleftline = []
            lanerightline = []
            for l in r['lanes']:
                laneleftline.append(l['left_line_id'])
                lanerightline.append(l['right_line_id'])
            for li in r['lines']:
                if li['id'] in laneleftline:
                    for nu in li['nurbs_curves']:
                        for k, v in paintDict.items():
                            if nu['nurbs_express_id'] in v['nurbs_express_id']:
                                paintDictNew[k]['left_line_id'] = li['id']
                if li['id'] in lanerightline:
                    for nu in li['nurbs_curves']:
                        for k, v in paintDict.items():
                            if nu['nurbs_express_id'] in v['nurbs_express_id']:
                                paintDictNew[k]['right_line_id'] = li['id']

            for l in r['lanes']:
                BoundaryPaintsDict[l['id']] = {}
                BoundaryPaintsDict[l['id']]['leftBoundaryPaints'] = []
                BoundaryPaintsDict[l['id']]['rightBoundaryPaints'] = []
                for lf in paintDictNew.values():
                    if 'left_line_id' in lf.keys() and l['left_line_id'] == lf['left_line_id']:
                        BoundaryPaintsDict[l['id']]['leftBoundaryPaints'].append(lf['paint_id'])
                    if 'right_line_id' in lf.keys() and l['right_line_id'] == lf['right_line_id']:
                        BoundaryPaintsDict[l['id']]['rightBoundaryPaints'].append(lf['paint_id'])

    for i in BoundaryPaintsDict.values():
        i['leftBoundaryPaints'] = sorted(i['leftBoundaryPaints'])
        i['rightBoundaryPaints'] = sorted(i['rightBoundaryPaints'])
    # for debug
    # jsonStr = json.dumps(lanechangeDict, indent=4)
    # with open('/home/user/Documents/APISDK_python/API_TestSpace/test_API/1.json', 'w') as ff:
    #     ff.write(jsonStr)


def getObjectsFromJson_PB13(conf_path):
    jsonFiles = findJSONFiles(conf_path)
    global curveDict
    global lineCurveDict
    global roadDict
    global lanechangeDict
    global BoundaryPaintsDict

    for i in jsonFiles:
        with open(conf_path + i, 'r') as f:
            jsonContent = json.load(f)
        # get curve of every segment
        roadList = jsonContent['roads']
        for r in roadList:
            # build roadDict
            roadDict[r['id']] = {}
            if 'from_road_ids' in r.keys():
                roadDict[r['id']]['preLandscapes'] = r['from_road_ids']
            else:
                roadDict[r['id']]['preLandscapes'] = []
            if 'to_road_ids' in r.keys():
                roadDict[r['id']]['nextLandscapes'] = r['to_road_ids']
            else:
                roadDict[r['id']]['nextLandscapes'] = []
            # build lineCurveDict
            for j in r['lines']:
                lineCurveDict[j['id']] = {}
                for jj in j['nurbs_curves']:
                    lineCurveDict[j['id']][jj['nurbs_express_id']] = {}
                    lineCurveDict[j['id']][jj['nurbs_express_id']]['start'] = jj['start']
                    lineCurveDict[j['id']][jj['nurbs_express_id']]['end'] = jj['end']
                    lineCurveDict[j['id']][jj['nurbs_express_id']]['length'] = jj['length']
                    for n in r['nurbs_expresses']:
                        if n['id'] in lineCurveDict[j['id']].keys():
                            lineCurveDict[j['id']][jj['nurbs_express_id']]['knots'] = n['knots']
                            lineCurveDict[j['id']][jj['nurbs_express_id']]['paint_total_length'] = n[
                                'paint_total_length']
                            lineCurveDict[j['id']][jj['nurbs_express_id']]['control_points'] = n['control_points']
                            lineCurveDict[j['id']][jj['nurbs_express_id']]['reference_point'] = jsonContent[
                                'reference_point']
            # build lanechangeDict
            for k in r['lanes']:
                evps = k['evps']
                for kk in evps:
                    if 'evp_attributes' in kk.keys():
                        lanechangeDict[kk['id']] = {}
                        evp_attributes = kk['evp_attributes']
                        lanechangeDictTemp = {}
                        for ee in evp_attributes:
                            if ee['attribute_type'] == 'EVPATTRTYPE_LANE_CHANGE':
                                lanechangeDictTemp[ee['id']] = {}
                                # getLength() is null in SDK
                                lanechangeDictTemp[ee['id']]['length'] = 0.0
                                if ee['attribute'].split(',')[0] == '0':
                                    lanechangeDictTemp[ee['id']]['isLeftChangeAllowed'] = False
                                else:
                                    lanechangeDictTemp[ee['id']]['isLeftChangeAllowed'] = True
                                    if 'left_lane_id' in k.keys():
                                        lanechangeDictTemp[ee['id']]['left_dest_lane'] = k['left_lane_id']
                                    else:
                                        lanechangeDictTemp[ee['id']]['left_dest_lane'] = None
                                if ee['attribute'].split(',')[1] == '0':
                                    lanechangeDictTemp[ee['id']]['isRightChangeAllowed'] = False
                                else:
                                    lanechangeDictTemp[ee['id']]['isRightChangeAllowed'] = True
                                    if 'right_lane_id' in k.keys():
                                        lanechangeDictTemp[ee['id']]['right_dest_lane'] = k['right_lane_id']
                                    else:
                                        lanechangeDictTemp[ee['id']]['right_dest_lane'] = None
                                lanechangeDictTemp[ee['id']]['start_position'] = ee['start_position']
                                lanechangeDictTemp[ee['id']]['end_position'] = ee['end_position']
                                lanechangeDictTemp[ee['id']]['sourceLane'] = k['id']
                                lanechangeDict[kk['id']][ee['id']] = lanechangeDictTemp[ee['id']]

        # build BoundaryPaintsDict in visualizations
        paintDict = {}
        for v in jsonContent['visualizations']:
            if v['type'] == 'VISUALIZATIONTYPE_PAINT':
                paints = v['paints']
                for vv in paints:
                    paintDict[vv['line_id']] = {}
                    paintDict[vv['line_id']]['paint_id'] = vv['id']
        for r in roadList:
            for j in r['lines']:
                if j['id'] in paintDict.keys():
                    for jj in j['nurbs_curves']:
                        paintDict[j['id']]['nurbs_express_id'] = []
                        paintDict[j['id']]['nurbs_express_id'].append(jj['nurbs_express_id'])
        paintDictNew = copy.deepcopy(paintDict)
        for r in roadList:
            laneleftline = []
            lanerightline = []
            for l in r['lanes']:
                laneleftline.append(l['left_line_id'])
                lanerightline.append(l['right_line_id'])
            for li in r['lines']:
                if li['id'] in laneleftline:
                    for nu in li['nurbs_curves']:
                        for k, v in paintDict.items():
                            if nu['nurbs_express_id'] in v['nurbs_express_id']:
                                paintDictNew[k]['left_line_id'] = li['id']
                if li['id'] in lanerightline:
                    for nu in li['nurbs_curves']:
                        for k, v in paintDict.items():
                            if nu['nurbs_express_id'] in v['nurbs_express_id']:
                                paintDictNew[k]['right_line_id'] = li['id']

            for l in r['lanes']:
                BoundaryPaintsDict[l['id']] = {}
                BoundaryPaintsDict[l['id']]['leftBoundaryPaints'] = []
                BoundaryPaintsDict[l['id']]['rightBoundaryPaints'] = []
                for lf in paintDictNew.values():
                    if 'left_line_id' in lf.keys() and l['left_line_id'] == lf['left_line_id']:
                        BoundaryPaintsDict[l['id']]['leftBoundaryPaints'].append(lf['paint_id'])
                    if 'right_line_id' in lf.keys() and l['right_line_id'] == lf['right_line_id']:
                        BoundaryPaintsDict[l['id']]['rightBoundaryPaints'].append(lf['paint_id'])

    for i in BoundaryPaintsDict.values():
        i['leftBoundaryPaints'] = sorted(i['leftBoundaryPaints'])
        i['rightBoundaryPaints'] = sorted(i['rightBoundaryPaints'])


def getCurve(horizon):
    def getRoad(road):
        def curveDetail(lineID, curve):
            # RDBVehicleAPI::Curve::getKnots()
            curveID = lineID + '_' + curve.getID().split('*')[-1]
            assert curve.getKnots() == lineCurveDict[lineID][curveID]['knots'], ("curveID: ", curveID,
                                                                                 "curve.getKnots() not same; SDK return: ",
                                                                                 curve.getKnots(),
                                                                                 "json content: ",
                                                                                 lineCurveDict[lineID][
                                                                                     curve.getID().split('*')[0]][
                                                                                     'knots'])
            # RDBVehicleAPI::Curve::getControlPoints()
            controlPoints = curve.getControlPoints()
            for cp in controlPoints:
                index = controlPoints.index(cp)
                # print(curveDict[curve.getID()]['control_points'][index])
                assert cp.x == lineCurveDict[lineID][curveID]['control_points'][index]['x'], \
                    ("curveID: ", curve.getID(), "controlPoints.x not same; SDK return: ", cp.x, "json content: ",
                     lineCurveDict[lineID][curveID]['control_points'][index][0])
                assert cp.y == lineCurveDict[lineID][curveID]['control_points'][index]['y'], \
                    ("curveID: ", curve.getID(), "controlPoints.y not same; SDK return: ", cp.y, "json content: ",
                     lineCurveDict[lineID][curveID]['control_points'][index][1])
                assert cp.z == lineCurveDict[lineID][curveID]['control_points'][index]['z'], \
                    ("curveID: ", curve.getID(), "controlPoints.z not same; SDK return: ", cp.z, "json content: ",
                     lineCurveDict[lineID][curveID]['control_points'][index][2])
            # RDBVehicleAPI::Curve::getPaintTotalLength()
            assert curve.getPaintTotalLength() == lineCurveDict[lineID][curveID]['paint_total_length'], \
                ("curveID: ", curve.getID(), "curve.getPaintTotalLength() not same; SDK return: ",
                 curve.getPaintTotalLength(), "json content: ", lineCurveDict[lineID][curveID]['paint_total_length'])
            # RDBVehicleAPI::Curve::getLength() --> SDK length is calculated
            # assert curve.getLength() == lineCurveDict[lineID][curve.getID()]['length'], \
            #     ("curveID: ", curve.getID(), "curve.getLength() not same; SDK return: ", curve.getLength(),
            #      "json content: ", lineCurveDict[lineID][curve.getID()]['length'])
            # RDBVehicleAPI::Curve::getParamMin()
            assert curve.getParamMin() == lineCurveDict[lineID][curveID]['start'], ("curveID: ", curveID,
                                                                                    "curve.getParamMin() not same; SDK return: ",
                                                                                    curve.getParamMin(),
                                                                                    "json content: ",
                                                                                    lineCurveDict[lineID][curveID][
                                                                                        'start'])
            # RDBVehicleAPI::Curve::getParamMax()
            assert curve.getParamMax() == lineCurveDict[lineID][curveID]['end'], \
                ("curveID: ", curveID, "curve.getParamMax() not same; SDK return: ",
                 curve.getParamMax(), "json content: ", lineCurveDict[lineID][curveID]['end'])
            # RDBVehicleAPI::Curve::getReferencePoint()
            refPoint = curve.getReferencePoint()
            assert refPoint.lon == lineCurveDict[lineID][curveID]['reference_point']['lon'], \
                ('reference point lon is not the same: SDK return: ', refPoint.lon, 'json content: ',
                 lineCurveDict[lineID][curveID]['reference_point']['lon'])
            assert refPoint.lat == lineCurveDict[lineID][curveID]['reference_point']['lat'], \
                ('reference point lat is not the same: SDK return: ', refPoint.lat, 'json content: ',
                 lineCurveDict[lineID][curveID]['reference_point']['lat'])
            assert refPoint.alt == lineCurveDict[lineID][curveID]['reference_point']['alt'], \
                ('reference point alt is not the same: SDK return: ', refPoint.alt, 'json content: ',
                 lineCurveDict[lineID][curveID]['reference_point']['alt'])

        for lane in road.getLanes():
            leftline = lane.getLeftLaneBoundary()
            for curve in leftline.getCurves():
                curveDetail(leftline.getID(), curve)
            rightline = lane.getRightLaneBoundary()
            for curve in rightline.getCurves():
                curveDetail(rightline.getID(), curve)
            centerline = lane.getCenterLine()
            for curve in centerline.getCurves():
                curveDetail(centerline.getID(), curve)

    for rd in horizon.getLandscapes():
        getRoad(rd)
    for jc in horizon.getJunctions():
        for rd in jc.getLandscapes():
            getRoad(rd)


def getLaneChange(horizon):
    def getRoad(road):
        for lane in road.getLanes():
            for evp in lane.getEVPs():
                laneChanges = evp.getLaneChanges()
                for lc in laneChanges:
                    # check getStartPoint()
                    startPoint = lc.getStartPoint()
                    assert lanechangeDict[evp.getID()][lc.getID()]['start_position']['lat'] == startPoint.lat, \
                        ('evp_id: ', evp.getID(), 'lanechange_id: ', lc.getID(), 'Json start_position.lat', \
                         lanechangeDict[evp.getID()][lc.getID()]['start_position']['lat'], \
                         'SDK startPoint.lat: ', startPoint.lat)
                    assert lanechangeDict[evp.getID()][lc.getID()]['start_position']['lon'] == startPoint.lon, \
                        ('evp_id: ', evp.getID(), 'lanechange_id: ', lc.getID(), 'Json start_position.lat', \
                         lanechangeDict[evp.getID()][lc.getID()]['start_position']['lon'], \
                         'SDK startPoint.lon: ', startPoint.lon)
                    assert lanechangeDict[evp.getID()][lc.getID()]['start_position']['alt'] == startPoint.alt, \
                        ('evp_id: ', evp.getID(), 'lanechange_id: ', lc.getID(), 'Json start_position.lat', \
                         lanechangeDict[evp.getID()][lc.getID()]['start_position']['alt'], \
                         'SDK startPoint.alt: ', startPoint.alt)
                    # check getEndPoint()
                    endPoint = lc.getEndPoint()
                    assert lanechangeDict[evp.getID()][lc.getID()]['end_position']['lat'] == endPoint.lat, \
                        ('evp_id: ', evp.getID(), 'lanechange_id: ', lc.getID(), 'Json end_position.lat', \
                         lanechangeDict[evp.getID()][lc.getID()]['end_position']['lat'], \
                         'SDK endPoint.lat: ', endPoint.lat)
                    assert lanechangeDict[evp.getID()][lc.getID()]['end_position']['lon'] == endPoint.lon, \
                        ('evp_id: ', evp.getID(), 'lanechange_id: ', lc.getID(), 'Json end_position.lat', \
                         lanechangeDict[evp.getID()][lc.getID()]['end_position']['lon'], \
                         'SDK endPoint.lon: ', endPoint.lon)
                    assert lanechangeDict[evp.getID()][lc.getID()]['end_position']['alt'] == endPoint.alt, \
                        ('evp_id: ', evp.getID(), 'lanechange_id: ', lc.getID(), 'Json end_position.lat', \
                         lanechangeDict[evp.getID()][lc.getID()]['end_position']['alt'], \
                         'SDK endPoint.alt: ', endPoint.alt)
                    # check getLength()
                    assert lanechangeDict[evp.getID()][lc.getID()]['length'] == lc.getLength(), \
                        ('evp_id: ', evp.getID(), 'lanechange_id: ', lc.getID(), 'Json length: ', \
                         lc.getLength(), 'SDK length: ', lc.getLength())
                    # check isLeftChangeAllowed()
                    assert lanechangeDict[evp.getID()][lc.getID()]['isLeftChangeAllowed'] == lc.isLeftChangeAllowed(), \
                        ('evp_id: ', evp.getID(), 'lanechange_id: ', lc.getID(), 'Json isLeftChangeAllowed: ', \
                         lanechangeDict[evp.getID()][lc.getID()]['isLeftChangeAllowed'], \
                         'SDK isLeftChangeAllowed: ', lc.isLeftChangeAllowed())
                    # check getLeftDestLane()
                    if lc.isLeftChangeAllowed() is True:
                        left_dest_lane = PyRIVLogicMgr.PyUtils.getLeftDestLaneByLaneChange(lc)
                        if left_dest_lane is not None:
                            assert lanechangeDict[evp.getID()][lc.getID()]['left_dest_lane'] == left_dest_lane.getID(), \
                                ('evp_id: ', evp.getID(), 'lanechange_id: ', lc.getID(), 'Json left_dest_lane: ', \
                                 lanechangeDict[evp.getID()][lc.getID()]['left_dest_lane'], 'SDK left_dest_lane: ', \
                                 left_dest_lane.getID())
                    # check isRightChangeAllowed()
                    assert lanechangeDict[evp.getID()][lc.getID()]['isRightChangeAllowed'] == lc.isRightChangeAllowed(), \
                        ('evp_id: ', evp.getID(), 'lanechange_id: ', lc.getID(), 'Json isRightChangeAllowed: ', \
                         lanechangeDict[evp.getID()][lc.getID()]['isRightChangeAllowed'], \
                         'SDK isRightChangeAllowed: ', lc.isRightChangeAllowed())
                    # check getRightDestLane()
                    if lc.isRightChangeAllowed() is True:
                        right_dest_lane = PyRIVLogicMgr.PyUtils.getRightDestLaneByLaneChange(lc)
                        if right_dest_lane is not None:
                            assert lanechangeDict[evp.getID()][lc.getID()][
                                       'right_dest_lane'] == right_dest_lane.getID(), \
                                ('evp_id: ', evp.getID(), 'lanechange_id: ', lc.getID(), 'Json right_dest_lane: ', \
                                 lanechangeDict[evp.getID()][lc.getID()]['right_dest_lane'], 'SDK right_dest_lane: ', \
                                 right_dest_lane.getID())

                    # check getSourceLane()
                    source_lane = PyRIVLogicMgr.PyUtils.getSourceLaneByLaneChange(lc)
                    if source_lane is not None:
                        assert lanechangeDict[evp.getID()][lc.getID()]['sourceLane'] == source_lane.getID(), \
                            ('evp_id: ', evp.getID(), 'lanechange_id: ', lc.getID(), 'Json getSourceLane: ', \
                             lanechangeDict[evp.getID()][lc.getID()]['sourceLane'], 'SDK getSourceLane: ', \
                             source_lane.getID())
                    else:
                        print(lanechangeDict[evp.getID()][lc.getID()]['sourceLane'])

    for road in horizon.getLandscapes():
        getRoad(road)
    for jc in horizon.getJunctions():
        for rd in jc.getLandscapes():
            getRoad(rd)


def getLaneScapes(horizon):
    def getRoad(road):
        preRoadList = []
        nextRoadList = []
        preRoads = PyRIVLogicMgr.PyUtils.getPreLandscapesByLandscape(road)
        for pr in preRoads:
            preRoadList.append(pr.getID())
        # check preRoadList
        assert preRoadList == roadDict[road.getID()]['preLandscapes'], ("roadID: ", road.getID(),
                                                                        "preLandscapes not same; SDK return: ",
                                                                        preRoadList, "json content: ",
                                                                        roadDict[road.getID()]['preLandscapes'])
        nextRoads = PyRIVLogicMgr.PyUtils.getNextLandscapesByLandscape(road)
        for nr in nextRoads:
            nextRoadList.append(nr.getID())
        assert nextRoadList == roadDict[road.getID()]['nextLandscapes'], ("roadID: ", road.getID(),
                                                                          "nextLandscapes not same; SDK return: ",
                                                                          nextRoadList, "json content: ",
                                                                          roadDict[road.getID()]['nextLandscapes'])

    for road in horizon.getLandscapes():
        getRoad(road)
    for jc in horizon.getJunctions():
        for rd in jc.getLandscapes():
            getRoad(rd)


def getLane(horizon):
    def getRoad(road):
        for lane in road.getLanes():
            leftBoundaryPaints = lane.getLeftBoundaryPaints()
            rightBoundaryPaints = lane.getRightBoundaryPaints()
            SDK_leftBoundaryPaints = []
            SDK_rightBoundaryPaints = []
            for l in leftBoundaryPaints:
                SDK_leftBoundaryPaints.append(l.getID())
            for r in rightBoundaryPaints:
                SDK_rightBoundaryPaints.append(r.getID())
            # check len(leftBoundaryPaints)/len(rightBoundaryPaints)
            assert len(BoundaryPaintsDict[lane.getID()]['leftBoundaryPaints']) == len(leftBoundaryPaints), \
                ('laneID: ', lane.getID(), 'Json_leftBoundaryPaints.length is ', \
                 len(BoundaryPaintsDict[lane.getID()]['leftBoundaryPaints']), \
                 'SDK_leftBoundaryPaints.length is ', len(leftBoundaryPaints))
            assert len(BoundaryPaintsDict[lane.getID()]['rightBoundaryPaints']) == len(rightBoundaryPaints), \
                ('laneID: ', lane.getID(), 'Json_rightBoundaryPaints.length is ', \
                 len(BoundaryPaintsDict[lane.getID()]['rightBoundaryPaints']), \
                 'SDK_rightBoundaryPaints is ', len(rightBoundaryPaints))
            # check leftBoundaryPaints/rightBoundaryPaints
            assert set(BoundaryPaintsDict[lane.getID()]['leftBoundaryPaints']) == set(SDK_leftBoundaryPaints), \
                ('laneID: ', lane.getID(), 'Json_leftBoundaryPaints is ', \
                 BoundaryPaintsDict[lane.getID()]['leftBoundaryPaints'], \
                 'SDK_leftBoundaryPaints is ', SDK_leftBoundaryPaints)
            assert set(BoundaryPaintsDict[lane.getID()]['rightBoundaryPaints']) == set(SDK_rightBoundaryPaints), \
                ('laneID: ', lane.getID(), 'Json_rightBoundaryPaints is ', \
                 BoundaryPaintsDict[lane.getID()]['rightBoundaryPaints'], \
                 'SDK_rightBoundaryPaints is ', SDK_rightBoundaryPaints)

    for rd in horizon.getLandscapes():
        getRoad(rd)
    for jc in horizon.getJunctions():
        for rd in jc.getLandscapes():
            getRoad(rd)


def initHorizon(conf_path):
    mngr = PyRIVLogicMgr.RIVLogicMgr()
    conf = PyRIVLogicMgr.conf_t()
    conf.dbPath = conf_path
    conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
    conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_WHOLE_E
    mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_ERROR)
    return_code = mngr.initWithConf(conf)
    assert return_code == 0, ("initHorizon init: fail; return code: ", return_code)
    horizon_result = mngr.getWholeHorizon()
    assert horizon_result.errorCode == 0, ("initHorizon fail, API returned error code: ", horizon_result.errorCode)
    assert horizon_result.spHorizon != None, ("initHorizon fail, spHorizon is: ", horizon_result.spHorizon)
    getCurve(horizon_result.spHorizon)
    getLaneChange(horizon_result.spHorizon)
    getLaneScapes(horizon_result.spHorizon)
    getLane(horizon_result.spHorizon)
    mngr.destroy()


def resetGlobal():
    global curveDict
    global lineCurveDict
    global roadDict
    global lanechangeDict
    global BoundaryPaintsDict
    curveDict = {}
    lineCurveDict = {}
    roadDict = {}
    lanechangeDict = {}
    BoundaryPaintsDict = {}


# only debug this case
# if __name__=='__main__':
#     for db in TestDB.keys():
#         print('>>>>>>>>>>>>>>>>>>db under test: ', db, '>>>>>>>>>>>>>>>>>')
#         if 0.14 == TestDB[db]['proto_ver']:
#             getObjectsFromJson_PB14(TestDB[db]['dbPath'])
#         if 0.13 == TestDB[db]['proto_ver']:
#             getObjectsFromJson_PB13(TestDB[db]['dbPath'])
#         initHorizon(TestDB[db]['dbPath'])
#         resetGlobal()
#     print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Test Done')

def test_VehicleAPI():
    for db in TestDB.keys():
        print('>>>>>>>>>>>>>>>>>>db under test: ', db, '>>>>>>>>>>>>>>>>>')
        if 0.14 == TestDB[db]['proto_ver']:
            getObjectsFromJson_PB14(TestDB[db]['dbPath'])
        if 0.13 == TestDB[db]['proto_ver']:
            getObjectsFromJson_PB13(TestDB[db]['dbPath'])
        initHorizon(TestDB[db]['dbPath'])
        resetGlobal()
    print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Test Done')

if __name__=='__main__':
    test_VehicleAPI()
