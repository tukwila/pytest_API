/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Message.h
 * @brief  The class for message.
 *
 * Rivision History:
 *      Date              Submitter         Description
 *      2018.02.05        Tony Xiong		Initialization
 *******************************************************************************
 */
#ifndef COM_YGOMI_ROADDB_UTIL_MESSAGE_H_
#define COM_YGOMI_ROADDB_UTIL_MESSAGE_H_

#include <map>
#include <memory>
#include <boost/any.hpp>

namespace RDBVehicleAPI
{

class Message {
public:
    /**
     * Default Constructor
     *
     * Notes:
     * Id will be set to -1, and type will be set to -1.
     */
    Message();

    /**
     * Constructor
     *
     * @param id Id of the message, which is used to identify the message.
     * @param type Type of the message, which is used to map the message
     *             to its corresponding message handler.
     */
    Message(int id, int type);
    /**
     * Constructor
     *
     * @param id Id of the message, which is used to identify the message.
     * @param type Type of the message, which is used to map the message
     *             to its corresponding message handler.
     * @param body Body of the message, which contains the data of the message.
     */
    Message(int id, int type, const boost::any& body);

    /**
     * Destructor
     */
    virtual ~Message();

    /**
     * Get the message id.
     *
     * @return The id of the message.
     */
    int getId() const;
    /**
     * Get the message id.
     *
     * @return The type of the message.
     */
    int getType() const;

    /**
     * Get the message body.
     *
     * @return The pointer holding the data of the message.
     */
    boost::any getBody() const;

    /**
     * Get the property value by key.
     *
     * @param key The key of the property.
     * @return The value of the property given by key.
     */
    virtual boost::any getProperty(const char* key) const;

    /**
     * Get the property value by key.
     *
     * @param key The key of the property.
     * @return The pointer holding the value of the property
     *         given by key.
     */
    virtual boost::any getProperty(const std::string& key) const;

    /**
     * Set message body.
     *
     * @param body The pointer pointing to the data that the message
     *             holds. Note the pointer won't be freed when this
     *             message is destroyed.
     */
    void setBody(std::shared_ptr<void*> body) {_body = body;};

    /**
     * Set message property.
     *
     * @param key	The key of the property.
     * @param value The value of the property. Note the value pointer
     *              won't be freed when this message is destroyed.
     */
    virtual void setProperty(const char* key, const boost::any& value);

    /**
     * Set message property.
     *
     * @param key	The key of the property.
     * @param value The value of the property. Note the value pointer
     *              won't be freed when this message is destroyed.
     */
    virtual void setProperty(const std::string& key, const boost::any& value);

private:
    int _id;
    int _type;
    boost::any _body;
    std::map<std::string, boost::any> _propertyMap;

};

// } /* namespace util */
} /* namespace roaddb */


#endif /* COM_YGOMI_ROADDB_UTIL_MESSAGE_H_ */
