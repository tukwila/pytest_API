/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   DBAbstractLayer.h
 * @brief  Header file of class DBAbstractLayer.
 *******************************************************************************
 */

#ifndef  MDB_ABSTRACT_LAYER_H_
#define  MDB_ABSTRACT_LAYER_H_
#include <string>
#include <vector>


struct sqlite3;

namespace roadDBCore
{

typedef int32_t (* DB_CALLBACK)(void *, int , char **, char **);
/**
 *******************************************************************************
 * @class DBAbstractLayer  "roadDBCore/DBAbstractLayer.h"
 * @brief This is a class of connect control to physical database sqlite
 *
 * This class provide the access of methods to sqlite by sqlite3 lib.
 *******************************************************************************
 */
class DBAbstractLayer
{
public:
    DBAbstractLayer(const std::string& dbName);
    virtual ~DBAbstractLayer();

    /**
    ****************************************************************************
    * @brief executeSQL - This function is used to
    *
    *  <1> Parameter Description:
    *
    *  @param [In]  - sqlValue
    *
    *  @param [In]  - cb
    *
    *  @param [In]  - parameter
    *
    *  @return success return true, or return false
    *
    *  <2> Detailed Description:
    *
    *
    *  \ingroup roadDBCore
    ****************************************************************************
    */
    uint32_t executeSQL(const std::string& sqlValue,DB_CALLBACK cb = NULL,void* parameter = NULL);
    //uint32_t beginTranscation();
    uint32_t processTranscation(const std::string& tableName,const std::vector<std::vector<std::string>>& records);
    //uint32_t endTranscation();
    //uint32_t rollbackTranscation();

    /**
    ****************************************************************************
    * @brief getSelectData - This function is used to
    *
    *  <1> Parameter Description:
    *
    *  @param [In]  - sqlValue
    *
    *  @param [Out]  - nRow
    *
    *  @param [Out]  - nColumn
    *
    *  @return
    *
    *  <2> Detailed Description:
    *
    *
    *  \ingroup roadDBCore
    ****************************************************************************
    */
    uint32_t getSelectData(const std::string& sqlValue, int32_t& nRow, int32_t& nColumn, char **& dbResult);

    //get records from table, fix the bug, read '\0' from DB
    uint32_t getSelectData(const std::string& sqlValue, std::vector<std::vector<std::string>>& records);

    /**
    ****************************************************************************
    * @brief freeTable - This function is used to free the memory of table
    *
    *  <1> Parameter Description:
    *
    *  @return
    *
    *  <2> Detailed Description:
    *
    *
    *  \ingroup roadDBCore
    ****************************************************************************
    */
    void freeTable();

    /**
    ****************************************************************************
    * @brief isTableExist - This function is used to know the table is exist or not
    *
    *  <1> Parameter Description:
    *
    *  @return true table exist, or not
    *
    *  <2> Detailed Description:
    *
    *
    *  \ingroup roadDBCore
    ****************************************************************************
    */
    bool isTableExist(const std::string &tableName);

    /**
     ****************************************************************************
     * @brief Get name of database file.
     *
     * @param none
     *
     * @return name of database file
     *
     *  \ingroup MasterDB
     ****************************************************************************
     */
    std::string getDBFileName() const;
    void setDBFileName(const std::string &dbName);

private:

    /**
    ****************************************************************************
    * @brief openDB - This function is used to
    *
    *  <1> Parameter Description:
    *
    *  @param [In]  - createtablesql
    *
    *  @return success return true, or return false
    *
    *  <2> Detailed Description:
    *
    *
    *  \ingroup roadDBCore
    ****************************************************************************
    */
    uint32_t openDB();

    std::string  dbName_;
    char ** dbResult_;
    sqlite3 *sqliteDb_;
};
}

#endif
