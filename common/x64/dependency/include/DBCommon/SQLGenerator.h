/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   SQLGenerator.h
 * @brief  Header file of class SQLGenerator.
 *******************************************************************************
 */

#ifndef MDB_SQL_GENERATOR_H_
#define MDB_SQL_GENERATOR_H_
#include <string>
#include <vector>

namespace roadDBCore
{
/**
 *******************************************************************************
 * @class SQLGenerator  "DBCommon/SQLGenerator.h"
 * @brief This is a class to generate SQL sentence
 *
 * This class provide interface to generate defferent SQL sentence, which is
 * use to query the tables in database.
 *******************************************************************************
 */
class SQLGenerator
{
public:
    SQLGenerator(const std::string & tableName);

    /**
    ****************************************************************************
    * @brief generateSelectSQL - This function is used to
    *
    *  <1> Parameter Description:
    *
    *  @param [In]  - selectValue
    *
    *  @param [In]  - whereCondition
    *
    *  @param [In]  - nLimit
    *
    *  @return SQL centense
    *
    *  <2> Detailed Description:
    *
    *
    *  \ingroup HistoryDB
    ****************************************************************************
    */
    std::string generateSelectSQL(const std::vector<std::string>& selectValue,
                                  const std::vector<std::string>& whereCondition,
                                  int nLimit = 0);

    /**
    ****************************************************************************
    * @brief generateInsertSQL - This function is used to
    *
    *  <1> Parameter Description:
    *
    *  @param [In]  - insertRecord
    *
    *  @return SQL centense
    *
    *  <2> Detailed Description:
    *
    *
    *  \ingroup HistoryDB
    ****************************************************************************
    */
    std::string generateInsertSQL(const std::vector<std::string>& insertRecord);

    std::string generateInsertSQL(const std::vector<std::vector<std::string>>& insertRecord);

    /**
    ****************************************************************************
    * @brief generateQuerySQL - This function is used to
    *
    *  <1> Parameter Description:
    *
    *  @return SQL centense
    *
    *  <2> Detailed Description:
    *
    *
    *  \ingroup HistoryDB
    ****************************************************************************
    */
    std::string generateQuerySQL()
    {
        std::string sql("select * from ");
        sql.append(tableName_);

        return sql;
    }

    std::string  generateCountSQL();
    std::string  generateDeleteSQL(const std::string& whereCondition);
    std::string  generateDeleteSQL(const std::vector<std::string>& whereCondition);
private:

    /**
    ****************************************************************************
    * @brief getWhereCondition - This function is used to
    *
    *  <1> Parameter Description:
    *
    *  @param [In]  - whereCondition
    *
    *  @return SQL centense
    *
    *  <2> Detailed Description:
    *
    *
    *  \ingroup HistoryDB
    ****************************************************************************
    */
    std::string getWhereCondition(const std::vector<std::string>& whereCondition);
private:
    std::string tableName_;
};
}

#endif
