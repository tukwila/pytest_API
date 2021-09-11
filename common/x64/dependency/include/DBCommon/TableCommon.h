/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   TableCommon.h
 * @brief  Header file of class TableCommon.
 *******************************************************************************
 */

#ifndef  DB_OPERATION_H_
#define  DB_OPERATION_H_
#include "DBCommon/DBLayerManager.h"
#include "DBCommon/SQLGenerator.h"
#include "typeDef.h"

#include <vector>

namespace roadDBCore
{
/**
 *******************************************************************************
 * @class TableCommon  "DBCommon/TableCommon.h"
 * @brief This is a table operation interface class.
 *
 * This class provide the basic operations of one table in database.
 *******************************************************************************
 */
class TableCommon
{
public:

    TableCommon(const std::string &dbFileName,
                const std::string &tableName,
                const std::string &createTable = "");
    /**
    ****************************************************************************
    * @brief insertRecord - This function is used to insert one record
    *
    *  <1> Parameter Description:
    *
    *  @param [In]  - record
    *
    *  @return success return 0, or return error code
    *
    *  <2> Detailed Description:
    *
    *
    *  \ingroup MasterDB
    ****************************************************************************
    */
    uint32_t insertRecord(const std::vector<std::string>& record);

    /**
    ****************************************************************************
    * @brief insertRecord - This function is used to insert multi records
    *
    *  <1> Parameter Description:
    *
    *  @param [In]  - records   the record set of to be inserted
    *
    *  @return success return 0, or return error code
    *
    *  <2> Detailed Description:
    *
    *
    *  \ingroup MasterDB
    ****************************************************************************
    */
    uint32_t insertRecord(const std::vector<std::vector<std::string>>& records);

    /**
    ****************************************************************************
    * @brief getRecord - This function is used to get record by condition;
    *
    *  <1> Parameter Description:
    *
    *  @param [In]   whereCondition  SQL where statement used by SQLGenerator
    *
    *  @param [In]  - dbResult  data of record
    *
    *  @param [Out]  - row     row of record
    *
    *
    *  @return success return 0, or return error code
    *
    *  <2> Detailed Description:
    *
    *
    *  \ingroup MasterDB
    ****************************************************************************
    */    
    //*** USE THE BELOW TWO INTERFACES
    uint32_t getRecord(const std::vector<std::string> &whereCondtion,
                   std::vector<std::vector<std::string>>& records);
    uint32_t getRecord(const std::vector<std::string>& selectValue,
                     const std::vector<std::string> &whereCondtion,
                     std::vector<std::vector<std::string>>& records);

    /**
    ****************************************************************************
    * @brief deleteRecord - This function is used to delete multi records
    *
    *  <1> Parameter Description:
    *
    *  @param [In]  - records   the record set of to be deleted
    *
    *  @return success return 0, or return error code
    *
    *  <2> Detailed Description:
    *
    *
    *  \ingroup MasterDB
    ****************************************************************************
    */    
    uint32_t deleteRecord(std::vector<std::string>& where);
    uint32_t deleteRecord(const std::string& where);

    SegmentID_t getSegmentID();
    virtual ~TableCommon(){};

protected:
    SQLGenerator                        generator_;
    std::shared_ptr<DBAbstractLayer>    dbLayer_;
    std::string                         tableName_;

};

struct TableBuffer
{
public:
    TableBuffer(DBAbstractLayer &layer):
    buf(NULL),
    dbLayer(layer)
    {

    }
    ~TableBuffer()
    {
        if (NULL != buf)
        {
            dbLayer.freeTable();
        }
    }

    char ** buf;  //store table record pointer;
    DBAbstractLayer &dbLayer;
};

}
#endif
