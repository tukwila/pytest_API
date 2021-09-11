/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   LogWrapper.h
 * @brief  log Api
 *******************************************************************************
 */


#ifndef LOGWRAPPER_H__
#define LOGWRAPPER_H__

#include <string>
#include <sstream>
extern int g_iLogWrapperValidLevel;


//Log args
enum LOG_SINK
{
    enum_CONSOLE = 1, //log console output
    enum_DISK_DAILY = 2, // log disk file daily output
    enum_DISK_ROTAT = 4, // log disk file rotating output
    enum_DISK    = enum_DISK_DAILY //log disk file output
};

enum LOG_LEVEL
{
    enum_TRACE    = 0,
    enum_DEBUG    = 1,
    enum_INFO     = 2,
    enum_WARN     = 3,
    enum_ERROR    = 4,
    enum_CRITICAL = 5,
    enum_LOGOFF   = 6   //if log level set enum_LOGOFF,no log info output
};

/************************InitLog*************************
 ******Brief:Initialize log library
 ******Args Descriptions:
 log_file_name :
    1. set the name of disk file;
    2. default:"log_file"
 log_module_name :
    1. set the module name of log info,will write in every line;
    2. default:"logger"
 log_output_level :
    1. 0-trace,1-debug,2-info,3-warning,4-error,5-critical,6-log off;
    2. which level is less than log_output_level value,it will not output
    3. default:0
 log_output_sink :
    1. 1-console output,2-disk output,1|2-console and disk output:
    2. default:1|2

 *******
*********************************************************/
void InitLog(
        const std::string & log_file_name,
        const std::string & log_module_name,
        LOG_LEVEL log_output_level = enum_TRACE,
        LOG_SINK log_output_sink = static_cast<LOG_SINK>(enum_CONSOLE|enum_DISK),
        std::size_t rotate_max_size = 500*1048576,
        std::size_t rotate_max_files = 3,
        LOG_LEVEL keyInfo_log_output_level = enum_LOGOFF);
/************************InitLog*************************
 ******Brief:Uninitialize log library
 *******
*********************************************************/
void UninitLog();


/**************************************************************************//**
 @Function      TakeOverStderr()

 @Description   Take over the log printed in stderr
                Call this function after InitLog

 @Param[in]     None

 @Return        None

 @Cautions      This may cause rotate_max_size(in InitLog) no longer take effect accurately,
                    data'size which printed by Non-LogWrapper will not statistic in.
*//***************************************************************************/
void TakeOverStderr();


/************************toStream*************************
 ******Brief:turn type T to string
 ******
*********************************************************/
template<class T>
std::string toStream( const T& t)
{
    std::ostringstream oss;
    oss << t;
    return oss.str();
}

class LogWrapperImpl;
class LogWrapper
{
public:
    LogWrapper(const std::string& file,int line,int log_level);
	LogWrapper(const std::string& moduleName,const std::string& file,int line,int log_level);	
	LogWrapper(const std::string& moduleName,const std::string& file,const std::string& function,int line,int log_level);
    ~LogWrapper();
public: 
    LogWrapper& operator<<(int value);
    LogWrapper& operator<<(unsigned value);
    LogWrapper& operator<<(long value);
    LogWrapper& operator<<(unsigned long value);
    LogWrapper& operator<<(long long value);
    LogWrapper& operator<<(unsigned long long value);
    LogWrapper& operator<<(float value);
    LogWrapper& operator<<(double value);
    LogWrapper& operator<<(long double value);
    LogWrapper& operator<<(char value);
    LogWrapper& operator<<(const std::string &value);
    LogWrapper& operator<<(const char *value);
    LogWrapper& operator<<(const void * const value);
    LogWrapper& operator<<(const std::ostringstream& oss);

private:
    LogWrapper(const LogWrapper&);
    LogWrapper& operator=(const LogWrapper&);

private:
    LogWrapperImpl * impl_ptr_ = nullptr;
};

class EnterLeaveFunction
{
    public:
        EnterLeaveFunction(const std::string& functinName,const std::string& file,int line)
        :functinName_(functinName),
         file_(file),
         line_(line)
        {
           LogWrapper    log(file_,line_,1);
           log<<"[SYS] ENTER FUNCTION: "<<functinName_;
        }
        ~EnterLeaveFunction()
        {
           LogWrapper    log(file_,line_,1);
           log<<"[SYS] LEAVE FUNCTION: "<<functinName_;
        }
    private:
       std::string   functinName_;
       std::string   file_;
       int           line_;
};

#define ENTER_FUNCTION   \
    EnterLeaveFunction  fun(__FUNCTION__,__FILE__,__LINE__)
//Marco
#define LOG_TRACE           LogWrapper(__FILE__,__LINE__,0)
#define LOG_DEBUG           LogWrapper(__FILE__,__LINE__,1)
#define LOG_INFO            LogWrapper(__FILE__,__LINE__,2)
#define LOG_WARN            LogWrapper(__FILE__,__LINE__,3)
#define LOG_ERROR           LogWrapper(__FILE__,__LINE__,4)
#define LOG_CRITICAL        LogWrapper(__FILE__,__LINE__,5)

//SYSTEM Macro
#define SYS_MODULE_NAME    "[SYS] "
#define SYS_LOG_TRACE       LogWrapper(SYS_MODULE_NAME,__FILE__,__FUNCTION__,__LINE__,0)
#define SYS_LOG_DEBUG       LogWrapper(SYS_MODULE_NAME,__FILE__,__FUNCTION__,__LINE__,1)
#define SYS_LOG_INFO        LogWrapper(SYS_MODULE_NAME,__FILE__,__FUNCTION__,__LINE__,2)
#define SYS_LOG_WARN        LogWrapper(SYS_MODULE_NAME,__FILE__,__FUNCTION__,__LINE__,3)
#define SYS_LOG_ERROR       LogWrapper(SYS_MODULE_NAME,__FILE__,__FUNCTION__,__LINE__,4)
#define SYS_LOG_CRITICAL    LogWrapper(SYS_MODULE_NAME,__FILE__,__FUNCTION__,__LINE__,5)
#define SYS_LOG_INFO_BEGIN  SYS_LOG_INFO << "##########BEGIN##########"
#define SYS_LOG_INFO_END    SYS_LOG_INFO << "##########END##########"
#define SYS_LOG_DEBUG_BEGIN SYS_LOG_DEBUG << "##########BEGIN##########"
#define SYS_LOG_DEBUG_END   SYS_LOG_DEBUG << "##########END##########"

#define SYS_LOG_INFO_PROCESS(str) SYS_LOG_INFO << "==========" << str << "=========="
#define SYS_LOG_DEBUG_PROCESS(str) SYS_LOG_DEBUG << "==========" << str << "=========="

//COMMON Macro
#define COM_MODULE_NAME    "[COM] "
#define COM_LOG_TRACE       LogWrapper(COM_MODULE_NAME,__FILE__,__LINE__,0)
#define COM_LOG_DEBUG       LogWrapper(COM_MODULE_NAME,__FILE__,__LINE__,1)
#define COM_LOG_INFO        LogWrapper(COM_MODULE_NAME,__FILE__,__LINE__,2)
#define COM_LOG_WARN        LogWrapper(COM_MODULE_NAME,__FILE__,__LINE__,3)
#define COM_LOG_ERROR       LogWrapper(COM_MODULE_NAME,__FILE__,__LINE__,4)
#define COM_LOG_CRITICAL    LogWrapper(COM_MODULE_NAME,__FILE__,__LINE__,5)

//SAM Macro
#define SAM_MODULE_NAME    "[SAM] "
#define SAM_LOG_TRACE       LogWrapper(SAM_MODULE_NAME,__FILE__,__LINE__,0)
#define SAM_LOG_DEBUG       LogWrapper(SAM_MODULE_NAME,__FILE__,__LINE__,1)
#define SAM_LOG_INFO        LogWrapper(SAM_MODULE_NAME,__FILE__,__LINE__,2)
#define SAM_LOG_WARN        LogWrapper(SAM_MODULE_NAME,__FILE__,__LINE__,3)
#define SAM_LOG_ERROR       LogWrapper(SAM_MODULE_NAME,__FILE__,__LINE__,4)
#define SAM_LOG_CRITICAL    LogWrapper(SAM_MODULE_NAME,__FILE__,__LINE__,5)

//SLAM Macro
#define SLAM_MODULE_NAME   "[SLAM] "
#define SLAM_LOG_TRACE      LogWrapper(SLAM_MODULE_NAME,__FILE__,__LINE__,0)
#define SLAM_LOG_DEBUG      LogWrapper(SLAM_MODULE_NAME,__FILE__,__LINE__,1)
#define SLAM_LOG_INFO       LogWrapper(SLAM_MODULE_NAME,__FILE__,__LINE__,2)
#define SLAM_LOG_WARN       LogWrapper(SLAM_MODULE_NAME,__FILE__,__LINE__,3)
#define SLAM_LOG_ERROR      LogWrapper(SLAM_MODULE_NAME,__FILE__,__LINE__,4)
#define SLAM_LOG_CRITICAL   LogWrapper(SLAM_MODULE_NAME,__FILE__,__LINE__,5)

//LOC Macro
#define LOC_MODULE_NAME    "[LOC] "
#define LOC_LOG_TRACE       LogWrapper(LOC_MODULE_NAME,__FILE__,__LINE__,0)
#define LOC_LOG_DEBUG       LogWrapper(LOC_MODULE_NAME,__FILE__,__LINE__,1)
#define LOC_LOG_INFO        LogWrapper(LOC_MODULE_NAME,__FILE__,__LINE__,2)
#define LOC_LOG_WARN        LogWrapper(LOC_MODULE_NAME,__FILE__,__LINE__,3)
#define LOC_LOG_ERROR       LogWrapper(LOC_MODULE_NAME,__FILE__,__LINE__,4)
#define LOC_LOG_CRITICAL    LogWrapper(LOC_MODULE_NAME,__FILE__,__LINE__,5)

// For SDOR
#define SDOR_MODULE_NAME   "[SDOR] "
#define SDOR_LOG_TRACE      LogWrapper(SDOR_MODULE_NAME,__FILE__,__LINE__,0)
#define SDOR_LOG_DEBUG      LogWrapper(SDOR_MODULE_NAME,__FILE__,__LINE__,1)
#define SDOR_LOG_INFO       LogWrapper(SDOR_MODULE_NAME,__FILE__,__LINE__,2)
#define SDOR_LOG_WARN       LogWrapper(SDOR_MODULE_NAME,__FILE__,__LINE__,3)
#define SDOR_LOG_ERROR      LogWrapper(SDOR_MODULE_NAME,__FILE__,__LINE__,4)
#define SDOR_LOG_CRITICAL   LogWrapper(SDOR_MODULE_NAME,__FILE__,__LINE__,5)

//FOR TOOL
#define TOOL_MODULE_NAME   "[TOOL] "
#define TOOL_LOG_TRACE      LogWrapper(TOOL_MODULE_NAME,__FILE__,__LINE__,0)
#define TOOL_LOG_DEBUG      LogWrapper(TOOL_MODULE_NAME,__FILE__,__LINE__,1)
#define TOOL_LOG_INFO       LogWrapper(TOOL_MODULE_NAME,__FILE__,__LINE__,2)
#define TOOL_LOG_WARN       LogWrapper(TOOL_MODULE_NAME,__FILE__,__LINE__,3)
#define TOOL_LOG_ERROR      LogWrapper(TOOL_MODULE_NAME,__FILE__,__LINE__,4)
#define TOOL_LOG_CRITICAL   LogWrapper(TOOL_MODULE_NAME,__FILE__,__LINE__,5)

//FOR VAPI
#define VAPI_MODULE_NAME   "[VAPI] "
#define VAPI_LOG_TRACE      LogWrapper(VAPI_MODULE_NAME,__FILE__,__LINE__,0)
#define VAPI_LOG_DEBUG      LogWrapper(VAPI_MODULE_NAME,__FILE__,__LINE__,1)
#define VAPI_LOG_INFO       LogWrapper(VAPI_MODULE_NAME,__FILE__,__LINE__,2)
#define VAPI_LOG_WARN       LogWrapper(VAPI_MODULE_NAME,__FILE__,__LINE__,3)
#define VAPI_LOG_ERROR      LogWrapper(VAPI_MODULE_NAME,__FILE__,__LINE__,4)
#define VAPI_LOG_CRITICAL   LogWrapper(VAPI_MODULE_NAME,__FILE__,__LINE__,5)

//error code
std::string errorCode(unsigned value);


#endif //LOGWRAPPER_H__
