/**
 *******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB AG. 2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 ******************************************************************************
 * @file   StdinWatcher.hpp
 * @brief  Interface of StdinWatcher 
 *******************************************************************************
 */


#ifndef STDIN_WATCHER_H
#define STDIN_WATCHER_H 

#include <string>
#include <functional>
#include <memory>

/**
 * @brief A sigleton used to watch stdin
 */
class StdinWatcher 
{
    public:
        using CMDActionCB = std::function<void()>;

    public:
        virtual ~StdinWatcher() = 0;
        virtual bool start() = 0;
        virtual bool stop() = 0;

        virtual std::string getCmd() = 0;
        virtual void setStopCmdActionCB(CMDActionCB cb) = 0;
        virtual void setParentProcessExitActionCB(CMDActionCB cb) = 0;

        friend class StdinWatcherCreater;
    protected:
        StdinWatcher();

    private:
        StdinWatcher(const StdinWatcher&) = delete;
        StdinWatcher& operator=(const StdinWatcher&) = delete;
};

using  StdinWatcherPtr = std::shared_ptr<StdinWatcher>; 

class StdinWatcherCreater
{
    public :
        StdinWatcherPtr operator()();
};

#endif