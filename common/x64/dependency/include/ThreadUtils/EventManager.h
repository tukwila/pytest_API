/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2018-2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   EventManager.h
 * @brief  Definition of event manager
 *******************************************************************************
 */

#include <map>
#include <deque>
#include <functional>           // std::bind(), std::function<>
#include <algorithm>            // std::count_if()
#include <memory>               // std::shared_ptr
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <atomic>
#include <thread>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include "ThreadUtils/EventCommon.h"
#include "utility/SingletonManager.h"



#ifndef EVENT_MANAGER
#define EVENT_MANAGER

namespace roadDBCore
{

/**
 *******************************************************************************
 * @class EventManager
 *
 * @brief Support push both synchronous and asynchronous events.
 *  1. Synchronous event: Event would be processed directly by corresponding
 *     registered handlers in event-producer thread.
 *  2. Asynchronous event: First, Event would be saved into internal pending
 *     queue. Then event can be processed by other modules in their corresponding
 *     threads or be consumed by registered handlers in pre-created internal
 *     thread in asychronous mode.
 *
 *  NOTE: It's not thread safe to register or unregister event handlers dynamically
 *        when event manager is  working.
 *        Please use the derived class EnhancedEventManager for this feature.
 *******************************************************************************
 */
class EventManager
{

public:
    EventManager(): bExit_(false), ncacheEventPostThrd_(100) {};

    ~EventManager() {};

    enum EVENT_PRIORITY_E: uint8_t
    {
        EVENT_PRIORITY_LOW_E = 0,
        EVENT_PRIORITY_HIGH_E,
        EVENT_PRIORITY_MAX_E
    };

    using EventHandler_t = std::function<bool (const std::shared_ptr<EventBase_t> &)>;
    template<typename ModuleClass>
    using ModuleHandler_t = bool (ModuleClass::*)(const std::shared_ptr<EventBase_t> &);
    using HandlerIter_t = std::multimap<EventID_t, std::pair<EventHandler_t, bool>>::iterator;
    using HandlerSize_t = std::multimap<EventID_t, std::pair<EventHandler_t, bool>>::size_type;
    using EventSize_t = std::deque<std::shared_ptr<EventBase_t>>::size_type;

    /**
     *******************************************************************************
     * @brief registerx - Register handler for a given event
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - id   Event ID.
     *
     *  @param [In]  - handler   Callback function, used to process the event.
     *                 handlerAsync callback function wether run in synmode
     *
     *  @return  Handler iterator to flag position of the handler.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    template<typename CallableType>
    HandlerIter_t registerx(EventID_t id, CallableType handler, bool bHandlerSync = true)
    {
        std::lock_guard<std::mutex> lg(handlerMutex_);

        return mmapHandlers_.emplace(id, std::make_pair((EventHandler_t)(handler), bHandlerSync));
    }

    /**
     *******************************************************************************
     * @brief registerx - Register handler for a given event
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - id   Event ID.
     *
     *  @param [In]  - module   Module object
     *
     *  @param [In]  - handler   Callback function defined in Module class to
     *   process the event.
     *
     *  @return  Handler iterator to flag position of the handler.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    template<typename ModuleClass>
    HandlerIter_t registerx(EventID_t id, ModuleClass &module, ModuleHandler_t<ModuleClass> handler, bool bHandlerSync = true)
    {
        return registerx(id, std::bind(handler, &module, std::placeholders::_1), bHandlerSync);
    }

    /**
     *******************************************************************************
     * @brief unregister - Clear all handlers
     *
     *  <1> Parameter Description:
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void unregister()
    {
        std::lock_guard<std::mutex> lg(handlerMutex_);

        return mmapHandlers_.clear();
    }

    /**
     *******************************************************************************
     * @brief unregister - Unregister all handlers of an event specified by event ID.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - id   Event ID.
     *
     *  @return  The number of handlers has been unregistered.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    HandlerSize_t unregister(EventID_t id)
    {
        std::lock_guard<std::mutex> lg(handlerMutex_);

        return mmapHandlers_.erase(id);
    }

    /**
     *******************************************************************************
     * @brief unregister - Unregister an event handler
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - id   Event ID.
     *
     *  @param [In]  - it   Handler iterator.
     *
     *  @return  The number of handlers has been unregistered.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    HandlerSize_t unregister(EventID_t id, const HandlerIter_t &it);

    /**
     *******************************************************************************
     * @brief getHandlerSize - Get total number of event handlers
     *
     *  <1> Parameter Description:
     *
     *  @return  The total number of handlers.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    HandlerSize_t getHandlerSize() const
    {
        std::lock_guard<std::mutex> lg(handlerMutex_);

        return mmapHandlers_.size();
    }

    /**
     *******************************************************************************
     * @brief getHandlerSize - Get number of handlers for a given event
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - id   Event ID.
     *
     *  @return  The number of handlers.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    HandlerSize_t getHandlerSize(EventID_t id) const
    {
        std::lock_guard<std::mutex> lg(handlerMutex_);

        return mmapHandlers_.count(id);
    }

    /**
     *******************************************************************************
     * @brief getEventSize - Get total number of events in pending queue
     *
     *  <1> Parameter Description:
     *
     *  @return  The events number.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    EventSize_t getEventSize(EVENT_PRIORITY_E enExpectEventPri = EVENT_PRIORITY_LOW_E)
    {
        std::lock_guard<std::mutex> lg(eventMutex_);

        std::deque<std::shared_ptr<EventBase_t>> &eventQueue = getEventQueue(enExpectEventPri);

        return eventQueue.size();
    }

    /**
     *******************************************************************************
     * @brief getEventSize - Get number of a given event in pending queue
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - id   Event ID.
     *
     *  @return  The events number.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    EventSize_t getEventSize(EventID_t id, EVENT_PRIORITY_E enExpectEventPri = EVENT_PRIORITY_LOW_E) 
    {
        std::lock_guard<std::mutex> lg(eventMutex_);

        std::deque<std::shared_ptr<EventBase_t>> &eventQueue = getEventQueue(enExpectEventPri);

        return std::count_if(eventQueue.begin(),
                             eventQueue.end(),
                             [id] (const std::shared_ptr<EventBase_t> &spEvent)
                                 {return (spEvent && (spEvent->id == id));});
    }

    /**
     *******************************************************************************
     * @brief pushSyncEvent - Push synchronous event to be executed directly by all
     *  registered handlers.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - spEvent   Share pointer of a given event.
     *
     *  @return  True if success, or return false
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool pushSyncEvent(const std::shared_ptr<EventBase_t> &spEvent)
    {
        if (spEvent)
        {
            return execEvent(spEvent);
        }
        else
        {
            return false;
        }
    }

    /**
     *******************************************************************************
     * @brief pushSyncEvent - Push synchronous event to be executed directly by all
     *  registered handlers.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - event   A given event.
     *
     *  @return  True if success, or return false
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    /*
    bool pushSyncEvent(const EventBase_t &event)
    {
        return execEvent(event);
    }
    */

    /**
     *******************************************************************************
     * @brief pushAsyncEvent - Save Asynchronous event into pending queue
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - spEvent   Share pointer of a given event.
     *
     *  @return  True if success, or return false
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    bool pushAsyncEvent(const std::shared_ptr<EventBase_t> &spEvent, EVENT_PRIORITY_E enExpectEventPri = EVENT_PRIORITY_LOW_E)
    {
        cacheEvent(spEvent, enExpectEventPri);

        return true;
    }

    /**
     *******************************************************************************
     * @brief pullEvent - De-queue an event from the pending queue. If the pending
     *  queue is empty, then waiting until an event is ready.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - spEvent   Share pointer of a given event.
     *
     *  @return Share pointer of a given event if success, or return null share pointer
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    std::shared_ptr<EventBase_t> pullEvent(EVENT_PRIORITY_E enExpectEventPri = EVENT_PRIORITY_LOW_E);

    /**
     *******************************************************************************
     * @brief pullEvent - De-queue event from pending queue with specified maximum
     *  waiting time.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - waitMilliSeconds   Maximum waiting time measured by millisecond
     *
     *  @return Share pointer of a given event if success, or return null share pointer
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    std::shared_ptr<EventBase_t> pullEvent(uint32_t waitMilliSeconds, EVENT_PRIORITY_E enExpectEventPri = EVENT_PRIORITY_LOW_E);

    /**
     *******************************************************************************
     * @brief peekEvent - Get event from pending queue without pop up it.
     *
     *  <1> Parameter Description:
     *
     *  @return Share pointer of a given event if success, or return null share pointer
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    std::shared_ptr<EventBase_t> peekEvent(EVENT_PRIORITY_E enExpectEventPri = EVENT_PRIORITY_LOW_E);

    /**
     *******************************************************************************
     * @brief peekEvent - Get a certain event from pending queue without pop up it.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - id   Event ID.
     *
     *  @return Share pointer of a given event if success, or return null share pointer
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    std::shared_ptr<EventBase_t> peekEvent(EventID_t id, EVENT_PRIORITY_E enExpectEventPri = EVENT_PRIORITY_LOW_E);

    /**
     *******************************************************************************
     * @brief popEvent - Pop up an event from pending queue
     *
     *  <1> Parameter Description:
     *
     *  @return The number of event has been removed.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    EventSize_t popEvent(EVENT_PRIORITY_E enExpectEventPri = EVENT_PRIORITY_LOW_E);

    /**
     *******************************************************************************
     * @brief popEvent - Pop up a given event from pending queue, only remove one
     *  event at most
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - id   Event ID.
     *
     *  @return The number of event has been removed.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    EventSize_t popEvent(EventID_t id, EVENT_PRIORITY_E enExpectEventPri = EVENT_PRIORITY_LOW_E);

    /**
     *******************************************************************************
     * @brief removeEvent - Remove specified events from pending queue.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - id   Event ID.
     *
     *  @return The number of event has been removed.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    EventSize_t removeEvent(EventID_t id, EVENT_PRIORITY_E enExpectEventPri = EVENT_PRIORITY_LOW_E);

    /**
     *******************************************************************************
     * @brief clearEventList - Empty event pending queue.
     *
     *  <1> Parameter Description:
     *
     *  @return The number of event has been removed.
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void clearEventList()
    {
        std::lock_guard<std::mutex> lg(eventMutex_);

        dqPendingEventsLowPriority_.clear();
        dqPendingEventsHighPriority_.clear();
    }

    /**
     *******************************************************************************
     * @brief setExit - Configure the exit of the backend thread if event processing
     *  mode is resident.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - bExit   Control the exit of the backend thread if event
     *   processing mode is resident.
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void setExit(bool bExit)
    {
        cacheEvent(nullptr, EVENT_PRIORITY_LOW_E); // In order to wake up the sleeping thread pending in event processing
        cacheEvent(nullptr, EVENT_PRIORITY_HIGH_E); // In order to wake up the sleeping thread pending in event processing
        bExit_.store(bExit);
    }

    bool isExit()
    {
        return bExit_;
    }

    /**
     *******************************************************************************
     * @brief process - Process events in pending queue in backend thread.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - bResident   Be resident or not.
     *   Control event processing mode in backend thread.
     *   True -- Resident processing mode: Event would be looped to process in
     *       backend resident thread.
     *       If there is no event in pending queue, just waiting for the new event.
     *   False -- Quick processing mode: Just process all available events in
     *       pending queue in backend thread;
     *       If there is no event in pending queue, then exit from the thread.
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void process(bool bResident = false);

    bool EventEmpty(EVENT_PRIORITY_E enExpectEventPri = EVENT_PRIORITY_LOW_E);
    bool execEvent(const std::shared_ptr<EventBase_t> &spEvent,  bool bExpectSyncMode = true);

protected:
    void cacheEvent(const std::shared_ptr<EventBase_t> &spEvent, EVENT_PRIORITY_E enExpectEventPri = EVENT_PRIORITY_LOW_E);

    std::deque<std::shared_ptr<EventBase_t>> & getEventQueue(EVENT_PRIORITY_E enExpectEventPri);

    std::multimap<EventID_t, std::pair<EventHandler_t, bool>> &getHandlerMap() {return mmapHandlers_;}

    std::mutex &getHandlerMutex() {return handlerMutex_;}

    /**
     * No copy constructor.
     */
    EventManager(const EventManager&) = delete;

    /**
     * No assignment.
     */
    EventManager &operator=(const EventManager&) = delete;

private:
    /**
     * Use priority queue if we need priority for events.
     */
    std::deque<std::shared_ptr<EventBase_t>> dqPendingEventsLowPriority_;
    std::deque<std::shared_ptr<EventBase_t>> dqPendingEventsHighPriority_;

    mutable std::mutex eventMutex_;
    std::condition_variable eventCondLowPri_;
    std::condition_variable eventCondHighPri_;

    std::multimap<EventID_t, std::pair<EventHandler_t, bool>> mmapHandlers_;
    mutable std::mutex handlerMutex_;

    /**
     * Control the exit of the backend thread if event processing mode is resident.
     */
    std::atomic<bool> bExit_;

    uint32_t ncacheEventPostThrd_;
};

/**
 *******************************************************************************
 * @class EnhancedEventManager
 *
 * @brief Derived from EventManager, support all the features in event manager
 *        and extend to support registry and unregistry of event handlers
 *        dynamically when event manager is  working.
 *******************************************************************************
 */
class EnhancedEventManager: public EventManager
{
protected:
    // bool execEvent(const EventBase_t &event);
};

/**
 *******************************************************************************
 * @brief pushSyncEvent - Push synchronous event to be executed directly by all
 *  registered handlers.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - spEvent   Share pointer of a given event.
 *
 *  @return  True if success, or return false
 *
 *  <2> Detailed Description:
 *******************************************************************************
 */
inline bool pushSyncEvent(const std::shared_ptr<EventBase_t> &spEvent)
{
    return g_hSingleMgr->get<EventManager>().pushSyncEvent(spEvent);
}

/**
 *******************************************************************************
 * @brief pushSyncEvent - Push synchronous event to be executed directly by all
 *  registered handlers.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - event   A given event.
 *
 *  @return  True if success, or return false
 *
 *  <2> Detailed Description:
 *******************************************************************************
 */
/*
inline bool pushSyncEvent(const EventBase_t &event)
{
    return g_hSingleMgr->get<EventManager>().pushSyncEvent(event);
}
*/

/**
 *******************************************************************************
 * @brief pushAsyncEvent - Save Asynchronous event into pending queue
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - spEvent   Share pointer of a given event.
 *
 *  @return  True if success, or return false
 *
 *  <2> Detailed Description:
 *******************************************************************************
 */
inline bool pushAsyncEvent(const std::shared_ptr<EventBase_t> &spEvent, roadDBCore::EventManager::EVENT_PRIORITY_E enExpectEventPri = roadDBCore::EventManager::EVENT_PRIORITY_LOW_E)
{
    return g_hSingleMgr->get<EventManager>().pushAsyncEvent(spEvent, enExpectEventPri);
}

}// namespace roadDBCore




#endif // EVENT_MANAGER




