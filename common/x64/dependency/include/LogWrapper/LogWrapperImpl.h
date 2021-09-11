/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   LogWrapperImpl.h
 * @brief  implement of log
 *******************************************************************************
 */


#ifndef LOGWRAPPER_IMPL_H__
#define LOGWRAPPER_IMPL_H__

#include <string>
#include <memory>
#include <mutex>
#include <iostream>
#include <libgen.h>
#include "spdlog/spdlog.h"
#include "spdlog/fmt/bundled/format.h"

class LogWrapperImpl
{
public:
    LogWrapperImpl(const std::string& file,int line,int log_level)
        : file_(file)
        , line_(line)
        , log_level_(log_level)
    {
        if (file.empty())
        {
            return;
        }
        std::vector<char> vec(file.size()+1,0);
        memcpy(&vec.at(0),file.c_str(),file.size());
        char * base_name = basename(&vec.at(0));
        if (base_name)
        {//no need free base_name,base_name is point vec
            file_ = std::string(base_name);
        }
    }

    LogWrapperImpl(const std::string& file,const std::string function, int line,int log_level)
        : file_(file)
        , function_(function)
        , line_(line)
        , log_level_(log_level)
    {
        if (file.empty())
        {
            return;
        }
        std::vector<char> vec(file.size()+1,0);
        memcpy(&vec.at(0),file.c_str(),file.size());
        char * base_name = basename(&vec.at(0));
        if (base_name)
        {//no need free base_name,base_name is point vec
            file_ = std::string(base_name);
        }
    }

    ~LogWrapperImpl();

    static std::unique_ptr<spdlog::logger> init_logger();
    static void uninit_logger();
    static void flush();

public:
    template<class T>
    LogWrapperImpl& operator<<(const T& t);
private:
    LogWrapperImpl(const LogWrapperImpl &);
    LogWrapperImpl& operator=(const LogWrapperImpl &);

public:
    static std::string log_file_name_;
    static std::string log_module_name_;
    static int log_valid_level_;
    static int keyInfo_log_valid_level_;
    static int log_valid_sink_;
    static std::size_t rotate_max_size_;
    static std::size_t rotate_max_files_;
    static bool bUpdate;
private:
    std::string file_;
    std::string function_{""};
    int line_;
    int log_level_;
    fmt::MemoryWriter raw_;
    static std::unique_ptr<spdlog::logger> logger_ptr_;
    static std::mutex logger_lock_;
};

template<class T>
LogWrapperImpl& LogWrapperImpl::operator<<(const T& t)
{
    raw_ << t;
    return *this;
}

// Efficient Integer to String Conversions, by Matthew Wilson.
size_t convertHex(char buf[], uintptr_t value);

#endif //LOGWRAPPER_IMPL_H_n
