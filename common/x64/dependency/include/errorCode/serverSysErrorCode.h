/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017-2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   serverSysErrorCode.h
 * @brief  server system error code define.
 *******************************************************************************
 */
#ifndef SERVER_SYS_ERROR_CODE_H
#define SERVER_SYS_ERROR_CODE_H

#include "typeDef.h"
#include "errorCode/moduleMaskCode.h"

namespace roadDBCore
{
// COMMON ERROR
const uint32_t SERVER_ARGC_ERROR                      = SERVER_SYSTEM + 0X1;
const uint32_t SERVER_OPT_NOT_EXIST_ERROR             = SERVER_SYSTEM + 0X2;
const uint32_t SERVER_VERSION_PRINT                   = SERVER_SYSTEM + 0X3;
const uint32_t SERVER_HELP_PRINT                      = SERVER_SYSTEM + 0X4;
const uint32_t SERVER_API_PRINT                       = SERVER_SYSTEM + 0X5;
const uint32_t SERVER_JSON_FILE_ERROR                 = SERVER_SYSTEM + 0X6;
const uint32_t SERVER_ARGC_ERR                        = SERVER_SYSTEM + 0X7;

// SQLITE FILE READE OR WRITE ERROR
const uint32_t EXECUTE_SQL_FAILD                      = SERVER_SYSTEM + 0X8;
const uint32_t OPEN_DB_FAILED                         = SERVER_SYSTEM + 0X9;
const uint32_t SET_DB_SYNC_FAILED                     = SERVER_SYSTEM + 0XA;
const uint32_t COMMIT_DB_FAILED                       = SERVER_SYSTEM + 0XB;
const uint32_t ROLLBACK_DB_FAILED                     = SERVER_SYSTEM + 0XC;
const uint32_t DB_PREPARE_V2_FAILED                   = SERVER_SYSTEM + 0XD;
const uint32_t DB_RESET_FAILED                        = SERVER_SYSTEM + 0XE;
const uint32_t DB_BIND_TEXT_FAILED                    = SERVER_SYSTEM + 0XF;
const uint32_t DB_SQL_STEP_FAILED                     = SERVER_SYSTEM + 0X10;
const uint32_t DB_FINALIZE_FAILED                     = SERVER_SYSTEM + 0X11;
const uint32_t BEGIN_DB_TRANS_FAILED                  = SERVER_SYSTEM + 0X12;
const uint32_t SELECT_DB_DATA_FAILED                  = SERVER_SYSTEM + 0X13;
const uint32_t DELETE_NODE_FAILED                     = SERVER_SYSTEM + 0X14;
const uint32_t DELETE_NODE2NODE_FAILED                = SERVER_SYSTEM + 0X15;
const uint32_t DELETE_NODE2SECTION_FAILED             = SERVER_SYSTEM + 0X16;
const uint32_t INSERT_NODE2SECTION_FAILED             = SERVER_SYSTEM + 0X17;
const uint32_t GET_NODE2SECTION_FAILED                = SERVER_SYSTEM + 0X18;

const uint32_t GPS_FILE_ERROR                         = SERVER_SYSTEM + 0X19;
const uint32_t CALCULATE_PASS_SEG_FAILED              = SERVER_SYSTEM + 0X1A;
const uint32_t GPS_SAVE_JSON_FAILED                   = SERVER_SYSTEM + 0X1B;
const uint32_t LOAD_GPS_FAILED                        = SERVER_SYSTEM + 0X1C;
const uint32_t DB_FILE_NAME_ERROR                     = SERVER_SYSTEM + 0X1D;
const uint32_t SERVER_EXIT                            = SERVER_SYSTEM + 0X1E;
const uint32_t GET_PIECE2SECTION_FAILED               = SERVER_SYSTEM + 0X1F;
const uint32_t DB_GET_NODE_FAILED                     = SERVER_SYSTEM + 0X20;
const uint32_t DB_ADD_NODE_FAILED                     = SERVER_SYSTEM + 0X21;
const uint32_t DB_ADD_NODE2NODE_FAILED                = SERVER_SYSTEM + 0X22;
const uint32_t DB_DELETE_NODE                         = SERVER_SYSTEM + 0X23;
const uint32_t FIND_ISOLATE_NODES_FAILED              = SERVER_SYSTEM + 0X24;
const uint32_t UPDATE_ISOLATE_NODES_FAILED            = SERVER_SYSTEM + 0X25;
const uint32_t DB_DELETE_NODE2NODE_FAILED             = SERVER_SYSTEM + 0X26;
const uint32_t DB_DELETE_NODE_FAILED                  = SERVER_SYSTEM + 0X27;
const uint32_t DB_ADD_LINE_SET_GROUP_FAILED           = SERVER_SYSTEM + 0X28;
const uint32_t DB_DEL_LINE_SET_GROUP_FAILED           = SERVER_SYSTEM + 0X29;
const uint32_t DB_GET_LINE_FAILED                     = SERVER_SYSTEM + 0X2A;
const uint32_t GET_LINE_ID_FAILED                     = SERVER_SYSTEM + 0X2B;
const uint32_t CHECK_CONNECTED_LANES_FAILED           = SERVER_SYSTEM + 0X2C;
const uint32_t MDB_ADD_ROAD_SET                       = SERVER_SYSTEM + 0X2D;

// TRANSFER DATA ERROR
const uint32_t TRANS_REF_SEG_INVALID_TILE_ID          = SERVER_SYSTEM + 0X2E;
const uint32_t TRANS_REF_SEG_INVALID_SECTION_ID       = SERVER_SYSTEM + 0X2F;
const uint32_t TRANS_REF_SEG_SEGID_NOT_MATCH          = SERVER_SYSTEM + 0X30;
const uint32_t TRANS_REF_SEG_CAL_SEGID_FAILED         = SERVER_SYSTEM + 0X31;
const uint32_t TRANS_REF_SEG_OFFSET_LESS_FAILED       = SERVER_SYSTEM + 0X32;
const uint32_t TRANS_REF_SEG_OFFSET_MORE_FAILED       = SERVER_SYSTEM + 0X33;
const uint32_t TRANS_REF_SEG_TRANS_SECTION            = SERVER_SYSTEM + 0X34;
const uint32_t TRANS_NODE_OFFSET_FAILED               = SERVER_SYSTEM + 0X35;
const uint32_t SELECT_REF_SEG_EMPTY                   = SERVER_SYSTEM + 0X36;
const uint32_t SELECT_REF_SEG_GET_CENTER              = SERVER_SYSTEM + 0X37;
const uint32_t TRANS_SNIP_TILEID_INVALID              = SERVER_SYSTEM + 0X38;
const uint32_t TRANS_SNIP2ERF_FAILED                  = SERVER_SYSTEM + 0X39;
const uint32_t TRANS_SNIP_FROM_ERF_FAILED             = SERVER_SYSTEM + 0X3A;
const uint32_t GET_SEG_OFFSET                         = SERVER_SYSTEM + 0X3B;
const uint32_t SET_CDB_SEGID_FAILED                   = SERVER_SYSTEM + 0X3C;
const uint32_t GET_CDB_POINT_FAILED                   = SERVER_SYSTEM + 0X3D;
const uint32_t TRANS_SECTION_DETAILS2REFSEG_FAILD     = SERVER_SYSTEM + 0X3E;
const uint32_t COOR_TRANS_SNIPPET_FAILD               = SERVER_SYSTEM + 0X3F;
const uint32_t COOR_TRANS_KF_FAILD                    = SERVER_SYSTEM + 0X40;
const uint32_t COOR_TRANS_MP_FAILED                   = SERVER_SYSTEM + 0X41;
const uint32_t COOR_TRANS_LINE_FAILED                 = SERVER_SYSTEM + 0X42;
const uint32_t COOR_TRANS_LINEGROUP_FAILED            = SERVER_SYSTEM + 0X43;
const uint32_t COOR_TRANS_KFLOC_FAILED                = SERVER_SYSTEM + 0X44;
const uint32_t COOR_TRANS_TCW_FAILED                  = SERVER_SYSTEM + 0X45;
const uint32_t COOR_TRANS_TWC_FAILED                  = SERVER_SYSTEM + 0X46;
const uint32_t COOR_TRANS_POSITION_FAILED             = SERVER_SYSTEM + 0X47;
const uint32_t CAL_SECTION_SEGID_FAILED               = SERVER_SYSTEM + 0X48;
const uint32_t GET_GPSFILE_FROM_CONFIG_FAILED         = SERVER_SYSTEM + 0X49;
const uint32_t CUT_SKELETON_PIECE_FAILED              = SERVER_SYSTEM + 0X4A;

// SERVER PROCESS ERROR
const uint32_t INIT_SERVER_ERROR                      = SERVER_SYSTEM + 0X4B;
const uint32_t SERVER_STITCHING_ARGC_ERR              = SERVER_SYSTEM + 0X4C;
const uint32_t SERVER_READ_DB_ERROR                   = SERVER_SYSTEM + 0X4D;
const uint32_t SERVER_PROCESS_DB_ERROR                = SERVER_SYSTEM + 0X4E;
const uint32_t SERVER_WRITE_DB_ERROR                  = SERVER_SYSTEM + 0X4F;

const uint32_t ADD_JSON_STATUS_ERROR                  = SERVER_SYSTEM + 0X50;
const uint32_t ANALYSE_JSON_STATUS_ERROR              = SERVER_SYSTEM + 0X51;

const uint32_t ROAD_SKEKETEN_DB_INIT_ERROR            = SERVER_SYSTEM + 0X52;
const uint32_t ROAD_SEGMENT_SEG_ERROR                 = SERVER_SYSTEM + 0X53;
const uint32_t ROAD_INDEX_FROM_HEAD_FAILED            = SERVER_SYSTEM + 0X54;

const uint32_t SERVER_ALGO_INIT_FAILED                = SERVER_SYSTEM + 0X55;
const uint32_t WRITE_SNIPPETS_FAILED                  = SERVER_SYSTEM + 0X56;
const uint32_t SAVE_GPS_INFO_FAILED                   = SERVER_SYSTEM + 0X57;
const uint32_t PARSE_DGPS_FILE_FAILED                 = SERVER_SYSTEM + 0X58;
const uint32_t ROAD_SEG_ID_ERROR                      = SERVER_SYSTEM + 0X59;
const uint32_t GENERATE_ROAD_SKELETON_ID_ERROR        = SERVER_SYSTEM + 0X5A;

const uint32_t VEHICLE_PARSER_READ_STRING_FAILED      = SERVER_SYSTEM + 0X5B;
const uint32_t VEHICLE_PARSER_READ_FILE_FAILED        = SERVER_SYSTEM + 0X5C;

const uint32_t VIDEO_EXTRACT_INIT_ERROR               = SERVER_SYSTEM + 0X5D;
const uint32_t VIDEO_EXTRACT_EVALUATE_OW_FAILED       = SERVER_SYSTEM + 0X5E;

const uint32_t SERVER_SAVE_GPS_INFO_FAILED            = SERVER_SYSTEM + 0X5F;
const uint32_t SERVER_EXTRACT_FAILED                  = SERVER_SYSTEM + 0X60;
const uint32_t SERVER_EXTRACT_HEADER_FAILED           = SERVER_SYSTEM + 0X61;
const uint32_t SERVER_EXTRACT_MP_FAILED               = SERVER_SYSTEM + 0X62;
const uint32_t SERVER_EXTRACT_KF_FAILED               = SERVER_SYSTEM + 0X63;
const uint32_t SERVER_GET_MP_POS                      = SERVER_SYSTEM + 0X64;
const uint32_t SERVER_EXTRACT_LOAD_CONF_FILE          = SERVER_SYSTEM + 0X65;
const uint32_t SERVER_EXTRACT_SAVE_FILE               = SERVER_SYSTEM + 0X66;
const uint32_t SERVER_RUN_ALGO_ERROR                  = SERVER_SYSTEM + 0X67;

const uint32_t CAL_NODE_SEGID_FAILED                  = SERVER_SYSTEM + 0X68;
const uint32_t CT_ROADOBJECT_COEF_SIZE                = SERVER_SYSTEM + 0X69;
const uint32_t CT_ROADOBJECT_NOT_SUPPORTED            = SERVER_SYSTEM + 0X6A;
const uint32_t ADD_NOTE_NOT_IN_CENTER_SEG             = SERVER_SYSTEM + 0X6B;

const uint32_t VISUAL_TOOL_NO_DB_FILE                 = SERVER_SYSTEM + 0X6C;
const uint32_t VISUAL_TOOL_GET_SAMPLE_SECTION_FAIL    = SERVER_SYSTEM + 0X6D;
const uint32_t VISUAL_TOOL_GET_DETAIL_SECTION_FAIL    = SERVER_SYSTEM + 0X6E;

const uint32_t SERVER_CALC_PASS_SEGMENT_FAIL          = SERVER_SYSTEM + 0X6F;

//DB READ WRITE ERROR
const uint32_t DB_QUERY_ERROR                         = SERVER_SYSTEM + 0X70;
const uint32_t DB_INSERT_ERROR                        = SERVER_SYSTEM + 0X71;
const uint32_t DB_UPDATE_ERROR                        = SERVER_SYSTEM + 0X72;
const uint32_t DB_DELETE_ERROR                        = SERVER_SYSTEM + 0X73;
const uint32_t DB_VERSION_ERROR                       = SERVER_SYSTEM + 0x74;
const uint32_t DB_NEIGHBOUR_ERROR                     = SERVER_SYSTEM + 0x75;
const uint32_t DB_MAX_ID_ERROR                        = SERVER_SYSTEM + 0x76;
const uint32_t DB_ID_ERROR                            = SERVER_SYSTEM + 0x77;

// DB IMPLEMENT BUILD
const uint32_t  DIB_BACKENDDB_BUILD_FAIL              = SERVER_SYSTEM + 0X80;
const uint32_t  DIB_VEHICLEDB_BUILD_FAIL              = SERVER_SYSTEM + 0X81;
const uint32_t  DIB_DEBUGDB_BUILD_FAIL                = SERVER_SYSTEM + 0X82;


const uint32_t SERVER_CHECK_ROAD_FAIL                 = SERVER_SYSTEM + 0X90;
const uint32_t AVG_SLAM_TRACE_NULL                    = SERVER_SYSTEM + 0X91;

// UPDATE SECTION REFERENCE
const uint32_t SERVER_SECTION_REFERENCE               = SERVER_SYSTEM + 0X100;
const uint32_t SERVER_REF_NO_VERSION                  = SERVER_SECTION_REFERENCE + 0x01;
const uint32_t SERVER_REF_LOAD_JSON_FAIL              = SERVER_SECTION_REFERENCE + 0x02;
const uint32_t SERVER_REF_READ_JSON_FAIL              = SERVER_SECTION_REFERENCE + 0x03;
const uint32_t SERVER_REF_EMPTY_VERSION               = SERVER_SECTION_REFERENCE + 0x03;
const uint32_t SERVER_REF_MODE_ERROR                  = SERVER_SECTION_REFERENCE + 0X04;
const uint32_t SERVER_REF_VERSION_ERROR               = SERVER_SECTION_REFERENCE + 0x05;

// LOGIC DATA ACCESS
const uint32_t LOGIC_DA_PARSER_ERROR                  = SERVER_SYSTEM + 0X110;

// For Snippet Analyzer interface
const uint32_t SNIPPET_ANALYZER_OK                    = ROAD_DATABASE_OK;
const uint32_t SNIPPET_ANALYZER_INVALID_PARAM         = SERVER_SYSTEM + 0X130;
const uint32_t SNIPPET_ANALYZER_NOT_ENOUGH_PARAMS     = SERVER_SYSTEM + 0X131;
const uint32_t SNIPPET_ANALYZER_INPUTFILE_ERR         = SERVER_SYSTEM + 0X132;
const uint32_t SNIPPET_ANALYZER_INVALID_PAYLOAD_TYPE  = SERVER_SYSTEM + 0X133;
const uint32_t SNIPPET_ANALYZER_EXTRA_FAIL            = SERVER_SYSTEM + 0X134;
const uint32_t SNIPPET_ANALYZER_SAVE_FAIL             = SERVER_SYSTEM + 0X135;
const uint32_t SNIPPET_ANALYZER_ERR_LOAD_CONF         = SERVER_SYSTEM + 0X136;
const uint32_t SNIPPET_ANALYZER_ERR_VERSION           = SERVER_SYSTEM + 0X137;
const uint32_t SNIPPET_ANALYZER_ERR_HELP              = SERVER_SYSTEM + 0X138;
const uint32_t SNIPPET_ANALYZER_ERR_API               = SERVER_SYSTEM + 0X139;

//DATA VALIDATION CHECK
const uint32_t BACKEND_DATA_CHECK                  = SERVER_SYSTEM + 0X00FF000;
const uint32_t SDC_SECTION_SEGID_INVALID              = BACKEND_DATA_CHECK + 0X01;
const uint32_t SDC_SECTION_DBID_INVALID               = BACKEND_DATA_CHECK + 0X02;
const uint32_t SDC_SECTION_NODEID_INCONSISTENT        = BACKEND_DATA_CHECK + 0X03;
const uint32_t SDC_SECTION_CVMAT_INVALID              = BACKEND_DATA_CHECK + 0X04;
const uint32_t SDC_PIECE_LIST_EMPTY                   = BACKEND_DATA_CHECK + 0X05;
const uint32_t SDC_PIECE_DBID_INVALID                 = BACKEND_DATA_CHECK + 0X06;
const uint32_t SDC_PIECE_CVMAT_INVALID                = BACKEND_DATA_CHECK + 0X07;
const uint32_t SDC_PIECE_LRID_NOT_CONTAIN             = BACKEND_DATA_CHECK + 0X08;
const uint32_t SDC_PIECE_LRID_RELATION_INVALID        = BACKEND_DATA_CHECK + 0X09;
const uint32_t SDC_PIECE_LRID_EQUAL                   = BACKEND_DATA_CHECK + 0X0A;
const uint32_t SDC_KF_PID_NOT_CONTAIN                 = BACKEND_DATA_CHECK + 0X0B;
const uint32_t SDC_KF_CVMAT_INVALID                   = BACKEND_DATA_CHECK + 0X0C;
const uint32_t SDC_POINT_PID_NOT_CONTAIN              = BACKEND_DATA_CHECK + 0X0D;
const uint32_t SDC_POINT_CVMAT_INVALID                = BACKEND_DATA_CHECK + 0X0E;
const uint32_t SDC_TRAFFICSIGN_PID_NOT_CONTAIN        = BACKEND_DATA_CHECK + 0X0F;
const uint32_t SDC_LINE_PID_NOT_CONTAIN               = BACKEND_DATA_CHECK + 0X10;
const uint32_t SDC_SNIPPET_SECTIONID_INVALID          = BACKEND_DATA_CHECK + 0X11;
const uint32_t SDC_LINESET_PID_NOT_CONTAIN            = BACKEND_DATA_CHECK + 0X12;
const uint32_t SDC_MERGE_SIGN_DBID_INVALID            = BACKEND_DATA_CHECK + 0X13;
const uint32_t SDC_CATCH_DB_NODE_DATE_ERROR           = BACKEND_DATA_CHECK + 0X14;
const uint32_t SDC_CATCH_DB_NODE_NOT_EXIST            = BACKEND_DATA_CHECK + 0X15;
const uint32_t SDC_CATCH_DB_NODE_FIX_ERROR            = BACKEND_DATA_CHECK + 0X16;

const uint32_t SDC_POINT_OBSERVER_EMPTY               = BACKEND_DATA_CHECK + 0X17;
const uint32_t SDC_LINESET_GROUP_EMPTY                = BACKEND_DATA_CHECK + 0X18;
const uint32_t SDC_ALGO_LINESET_EMPTY                 = BACKEND_DATA_CHECK + 0X19;
const uint32_t SDC_LINESET_LINES_EMPTY                = BACKEND_DATA_CHECK + 0X1A;

const uint32_t SDC_PIECE_TRAJECTORY_EMPTY             = BACKEND_DATA_CHECK + 0X1B;
const uint32_t SDC_REFERENCE_EMPTY                    = BACKEND_DATA_CHECK + 0X1C;
const uint32_t SDC_KEYFRAME_LIST_EMPTY                = BACKEND_DATA_CHECK + 0X1D;
const uint32_t SDC_POINT_LIST_EMPTY                   = BACKEND_DATA_CHECK + 0X1E;
const uint32_t SDC_TRAFFICSIGN_OBSERVER_EMPTY         = BACKEND_DATA_CHECK + 0X1F;
const uint32_t SDC_LINE_LIST_EMPTY                    = BACKEND_DATA_CHECK + 0X20;
const uint32_t SDC_LINE_LEFT_EMPTY                    = BACKEND_DATA_CHECK + 0X21;
const uint32_t SDC_LINE_RIGHT_EMPTY                   = BACKEND_DATA_CHECK + 0X22;
const uint32_t SDC_ERR_SNIPPET_SEGMENTS               = BACKEND_DATA_CHECK + 0X23;
const uint32_t SDC_ERR_KEYFRAME_ID                    = BACKEND_DATA_CHECK + 0X24;
const uint32_t SDC_ERR_POINT_ID                       = BACKEND_DATA_CHECK + 0X25;

// For Restful DB interface
const uint32_t REST_DB_STATUS_OK                      = ROAD_DATABASE_OK;
const uint32_t REST_DB_STATUS_COMMON_ERROR            = SERVER_SYSTEM + 0X200;
const uint32_t REST_DB_STATUS_CONFIG_ERROR            = SERVER_SYSTEM + 0X201;
const uint32_t REST_DB_STATUS_CONNECTION_ERROR        = SERVER_SYSTEM + 0X202;
const uint32_t REST_DB_STATUS_PAYLOAD_ERROR           = SERVER_SYSTEM + 0X205;
const uint32_t REST_DB_STATUS_RESPONSE_ERROR          = SERVER_SYSTEM + 0X210;
const uint32_t REST_DB_STATUS_RESPONSE_PARSE_ERROR    = SERVER_SYSTEM + 0X211;
const uint32_t REST_DB_STATUS_ALLOC_MEMORY_ERROR      = SERVER_SYSTEM + 0X212;
const uint32_t REST_DB_STATUS_PARSE_TOKEN_FAIL        = SERVER_SYSTEM + 0X213;
const uint32_t REST_DB_STATUS_OPEN_FILE_FAIL          = SERVER_SYSTEM + 0X214;
const uint32_t REST_DB_INIT_FS_CLIENT_FAIL            = SERVER_SYSTEM + 0X215;
const uint32_t REST_DB_STATUS_NETWORK_ERROR           = SERVER_SYSTEM + 0X216;

}

#endif
