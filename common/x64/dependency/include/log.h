/**
*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG. 2016-2017
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
******************************************************************************
* @file   log.h
* @brief  Declaration of log
*******************************************************************************
*/
/*
 * Logger.h
 *
 *  Created on: Feb 29, 2016
 *      Author: lichie
 */

#ifndef LOG_H_
#define LOG_H_
#include <string>
#include <sstream>


namespace roadDB_logger
{

#ifndef ROADDB_LOGGER_LEVEL_DEF
#define ROADDB_LOGGER_LEVEL_DEF
enum Level :unsigned char{
    debug,
    info,
    warn,
    error
};
#endif

extern Level gs_iGlobleLogLevel;

void setLevel(const Level& lv);

void filterMessage(const char *tag, const std::string &message);

void doLogv2( const char * pTag, int iLine, const char * pFileName, Level logLvl, const char *format, ...) __attribute__((format(printf,5,6)));

#define DefaultModule "tag"
#define dl(format, args...) doLogv2( DefaultModule, __LINE__, __FILE__, roadDB_logger::debug, format, ## args )
#define il(format, args...) doLogv2( DefaultModule, __LINE__, __FILE__, roadDB_logger::info, format, ## args )
#define wl(format, args...) doLogv2( DefaultModule, __LINE__, __FILE__, roadDB_logger::warn, format, ## args )
#define el(format, args...) doLogv2( DefaultModule, __LINE__, __FILE__, roadDB_logger::error, format, ## args )

/*
You can self define the module tag log macro like this:
#define dldr(format, args...) doLogv2( "datareceiver", __LINE__, __FILE__, roadDB_logger::debug, format, ## args )
#define ildr(format, args...) doLogv2( "datareceiver", __LINE__, __FILE__, roadDB_logger::info, format, ## args )
#define wldr(format, args...) doLogv2( "datareceiver", __LINE__, __FILE__, roadDB_logger::warn, format, ## args )
#define eldr(format, args...) doLogv2( "datareceiver", __LINE__, __FILE__, roadDB_logger::error, format, ## args )

*/


//Interfaces not recommended
void e(const char *tag, const char *format, ...) __attribute__((format(printf,2,3)));
void w(const char *tag, const char *format, ...) __attribute__((format(printf,2,3)));
void d(const char *tag, const char *format, ...) __attribute__((format(printf,2,3)));
void i(const char *tag, const char *format, ...) __attribute__((format(printf,2,3)));

void WidebrightSegvHandler(int signum);
int signal_catch();

const char* NotNullStr( const char *a);

struct LoggerThreadInfo_t;

class CPPLog
{
public:
    CPPLog(const char * pTag, int iLine, const char * pFileName, Level logLvl);
    ~CPPLog();
    
    void *m_TIPtr;
    const char * m_pTag;
    int m_iLine;
    const char * m_pFileName;
    Level m_eLogLvl;
    std::stringstream &m_vStream;
};


#define dcpp if(roadDB_logger::gs_iGlobleLogLevel <= roadDB_logger::debug)CPPLog(DefaultModule, __LINE__, __FILE__, roadDB_logger::debug).m_vStream
#define icpp if(roadDB_logger::gs_iGlobleLogLevel <= roadDB_logger::info)CPPLog(DefaultModule, __LINE__, __FILE__, roadDB_logger::info).m_vStream
#define wcpp if(roadDB_logger::gs_iGlobleLogLevel <= roadDB_logger::warn)CPPLog(DefaultModule, __LINE__, __FILE__, roadDB_logger::warn).m_vStream
#define ecpp CPPLog(DefaultModule, __LINE__, __FILE__, roadDB_logger::error).m_vStream






}//name space COM_YGOMI end

#endif /* LOG_H_ */
