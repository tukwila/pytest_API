/**
 *******************************************************************************
 *                       RoadDB Confidential
 *       Copyright (c) Continental AG. 2016-2018, RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   roadDBJson.h
 * @brief  json file parse
 *******************************************************************************
 */

#ifndef ROADDB_JSON_H
#define ROADDB_JSON_H


#include <map>
#include <vector>
#include <iostream>
#include <sstream>
#include <istream>
#include <ostream>
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/all.hpp>
#include "typeDef.h"
#include "LogWrapper/LogWrapper.h"


//using namespace std;
using std::cout;
using std::vector;
using std::map;
using std::endl;


namespace roadDBCore
{
    enum JSON_TYPE_E
    {
        JSON_TYPE_SINGLE_E = 0,
        JSON_TYPE_ARRAY_E,
        JSON_TYPE_OBJECT_E,
        JSON_TYPE_MAX_E
    };

    class JsonParser
    {
    public:
        typedef boost::property_tree::ptree JsonTree;

        JsonParser();
        ~JsonParser();

        /* read related functions. */
        bool load(IN const std::string &filePath);
        bool loadFromStream(IN std::istringstream &ss);

        JsonTree& getRoot() {return jtree_;}

        /* get single value by keyPath. */
        template<typename T>
        bool getSingleValue(IN const std::string &keyPath, OUT T &value)
        {
            return getSingleValueFrom(jtree_, keyPath, value);
        }

        /**
         *******************************************************************************
         * @brief getSingleValueFrom
         *
         *  <1> Parameter Description:
         *
         *  @param [In]  - it
         *               - value
         *
         *  @param [Out]  - value
         *
         *  @return true or false
         *
         *  <2> Detailed Description:
         *******************************************************************************
         */
        template<typename T>
        bool getSingleValueFrom(IN const JsonTree &jt, IN const std::string &keyPath, OUT T&value)
        {
            if (!isSingleValue(jt, keyPath))
            {
                COM_LOG_ERROR << "The json path is not a single value";
                return false;
            }

            try
            {
                value = jt.get<T>(keyPath);
            }

            catch (boost::exception& e)
            {
                COM_LOG_ERROR << "Boost exception when getting value from json path: " << keyPath;
                COM_LOG_ERROR << boost::diagnostic_information(e);
                return false;
            }

            catch (std::exception &e)
            {
                COM_LOG_ERROR << "STD exception when getting value from json path: " << keyPath;
                COM_LOG_ERROR << e.what();
                return false;
            }

            catch (...)
            {
                COM_LOG_ERROR << "Unknow exception when getting value from json path: " << keyPath;
                return false;
            }

            return true;
        }

        /**
         *******************************************************************************
         * @brief getArrayOfSingleValuesFrom
         *
         *  <1> Parameter Description:
         *
         *  @param [In]  - it
         *               - keyPath
         *
         *  @param [Out]  - values
         *
         *  @return true or false
         *
         *  <2> Detailed Description:
         *******************************************************************************
         */
        template<typename T>
        bool getArrayOfSingleValuesFrom(IN const JsonTree &jt, IN const std::string &keyPath, OUT std::vector<T> &values)
        {
            try
            {
                const JsonTree &array = jt.get_child(keyPath);

                for (auto it = array.begin(); it != array.end(); ++it)
                {
                    if(!it->first.empty())
                    {
                        return false;
                    }

                    T element = it->second.get_value<T>();

                    values.push_back(element);
                }
            }

            catch (boost::exception& e)
            {
                COM_LOG_ERROR << "Boost exception when getting value from json path: " << keyPath;
                COM_LOG_ERROR << boost::diagnostic_information(e);
                return false;
            }

            catch (std::exception &e)
            {
                COM_LOG_ERROR << "STD exception when getting value from json path: " << keyPath;
                COM_LOG_ERROR << e.what();
                return false;
            }

            catch (...)
            {
                COM_LOG_ERROR << "Unknow exception when getting value from json path: " << keyPath;
                return false;
            }

            return true;
        }

        /**
         *******************************************************************************
         * @brief getArrayOfSingleValuesFrom
         *
         *  <1> Parameter Description:
         *
         *  @param [In]  - it
         *               - keyPath
         *
         *  @param [Out]  - objects
         *
         *  @return true or false
         *
         *  <2> Detailed Description:
         *******************************************************************************
         */
        template<typename T>
        bool getArrayOfSingleValuesFrom(IN const JsonTree &jt, IN const std::string &keyPath, OUT std::vector<std::vector<T>> &objects)
        {
            try
            {
                const JsonParser::JsonTree &level2Tree = jt.get_child(keyPath);
                std::vector<T> vecItem;

                objects.reserve(level2Tree.size());

                for (JsonTree::const_iterator it2 = level2Tree.begin(); it2 != level2Tree.end(); ++it2)
                {
                    if (!it2->first.empty())
                    {
                        return false;
                    }

                    vecItem.clear();

                    const JsonTree &level3Tree = it2->second;

                    for (JsonTree::const_iterator it3 = level3Tree.begin(); it3 != level3Tree.end(); ++it3)
                    {
                        if (!it3->first.empty())
                        {
                            return false;
                        }

                        vecItem.push_back(it3->second.get_value<T>());
                    }

                    objects.push_back(vecItem);
                }
            }

            catch (boost::exception& e)
            {
                COM_LOG_ERROR << "Boost exception when getting value from json path: " << keyPath;
                COM_LOG_ERROR << boost::diagnostic_information(e);
                return false;
            }

            catch (std::exception &e)
            {
                COM_LOG_ERROR << "STD exception when getting value from json path: " << keyPath;
                COM_LOG_ERROR << e.what();
                return false;
            }

            catch (...)
            {
                COM_LOG_ERROR << "Unknow exception when getting value from json path: " << keyPath;
                return false;
            }

            return true;
        }

        template<typename T>
        bool getArrayOfSingleValues(IN const std::string &keyPath, OUT std::vector<T> &values)
        {
            return getArrayOfSingleValuesFrom(jtree_, keyPath, values);
        }

        template<typename T>
        bool getArrayOfSingleValues(IN const std::string &keyPath, OUT std::vector<std::vector<T>> &values)
        {
            return getArrayOfSingleValuesFrom(jtree_, keyPath, values);
        }

        /**
         *******************************************************************************
         * @brief getArrayOfObject
         *
         *  <1> Parameter Description:
         *
         *  @param [In]  - keyPath
         *
         *  @param [Out]  - objects
         *
         *  @return true or false
         *
         *  <2> Detailed Description:
         *******************************************************************************
         */
        bool getArrayOfObject(IN const std::string &keyPath, OUT std::vector<std::map<std::string, std::string> > &objects);
        bool getArrayOfObject(IN const std::string &keyPath, OUT std::vector<std::map<std::string, std::vector<std::string>> > &objects);
        bool getArrayOfObject(IN const std::string &keyPath, OUT std::vector<std::vector<std::map<std::string, std::string>>> &objects);

        /**
         *******************************************************************************
         * @brief getArrayOfObjectFrom
         *
         *  <1> Parameter Description:
         *
         *  @param [In]  - jt
         *               - keyPath
         *
         *  @param [Out]  - objects
         *
         *  @return true or false
         *
         *  <2> Detailed Description:
         *******************************************************************************
         */
        bool getArrayOfObjectFrom(IN const JsonTree &jt, IN const std::string &keyPath, OUT std::vector<std::map<std::string, std::vector<std::string>> > &objects);
        bool getArrayOfObjectFrom(IN const JsonTree &jt, IN const std::string &keyPath, INOUT std::vector<std::map<std::string, std::string> > &objects);
        bool getArrayOfObjectFrom(IN const JsonTree &jt, IN const std::string &keyPath, OUT std::vector<std::vector<std::map<std::string, std::string>>> &objects);

        bool getObjectRecursion(const std::string& nodeName, std::map<std::string, std::string> &properties);
        bool getObject(IN const std::string &keyPath, OUT std::map<std::string, std::string> &properties);

        boost::optional<JsonTree &> getNode(IN const std::string &keyPath);
        bool getNode(IN const std::string &keyPath, OUT JsonTree &node);
        bool getNodeFrom(IN const JsonTree &jt, IN const std::string &keyPath, OUT JsonTree &node);
        bool getArrayOfNodeFrom(IN const JsonTree &jt, IN const std::string &keyPath, INOUT std::vector<JsonTree> &nodes);
        bool getKeyPathOfJsonTree(IN const JsonTree &jt, std::vector<std::string> &keyPaths);


        /*clear all the */
        void clear();

        /*
         *Add an object, addObject("root.obj1", const std::map<std::string, valueType>properties);
         *"obj1":
            {
                "property1 name": "property1 value",
                "property2 name": "property2 value",
                "property3 name": "property3 value",
            }
         */
        template<typename T>
        bool addObject(IN const std::string &keyPath, IN const std::map<std::string, T> &properties)
        {
            JsonTree newObject;
            auto it = properties.begin();

            try
            {
                for (; it != properties.end(); ++it)
                {
                    newObject.add(it->first, it->second);
                }

                addObject(keyPath, newObject);
            }

            catch (boost::exception& e)
            {
                COM_LOG_ERROR << "Boost exception when adding object to " << keyPath;
                COM_LOG_ERROR << boost::diagnostic_information(e);
                return false;
            }

            catch (std::exception &e)
            {
                COM_LOG_ERROR << "STD exception when getting object to " << keyPath;
                COM_LOG_ERROR << e.what();
                return false;
            }

            catch (...)
            {
                COM_LOG_ERROR << "Unknow exception when getting object to " << keyPath;
                return false;
            }

            return true;
        }

        JsonTree &addObjectTo(IN const std::string &keyPath, INOUT const JsonTree &obj, OUT JsonTree &jtTo);

        JsonTree &addObject(IN const std::string &keyPathOfParent, INOUT const JsonTree &obj);

        /*
         * Add a single value property for an object, addSingleValue("root.obj1.singleValueProperty", value);
         *"obj1":
            {
                "property1 name": "property1 value",
                "property2 name": "property2 value",
                "property3 name": "property3 value",
                "singleValueProperty": value;
            }
         */
        template<typename T>
        bool addSingleValue(IN const std::string &propertyKeyPath, IN T value)
        {
            return addSingleValueTo(propertyKeyPath, value, jtree_);
        }

        template<typename T>
        bool addSingleValueTo(IN const std::string &propertyKeyPath, IN T value, OUT JsonTree &jt)
        {
            try
            {
                jt.add(propertyKeyPath, value);
            }

            catch (boost::exception& e)
            {
                COM_LOG_ERROR << "Boost exception when adding value to " << propertyKeyPath;
                COM_LOG_ERROR << boost::diagnostic_information(e);
                return false;
            }

            catch (std::exception &e)
            {
                COM_LOG_ERROR << "STD exception when adding value to " << propertyKeyPath;
                COM_LOG_ERROR << e.what();
                return false;
            }

            catch (...)
            {
                COM_LOG_ERROR << "Unknow exception when adding value to " << propertyKeyPath;
                return false;
            }

            return true;
        }

        /*
         *Add an single value array property for an object, addArray("root.obj1.arrayProperty", const std::vector<T> &values);
         *"obj1":
            {
                "property1 name": "property1 value",
                "property2 name": "property2 value",
                "property3 name": "property3 value",
                "arrayProperty":
                [
                 "element0",
                 "element1"
                ]
            }
         */

        template<typename T>
        bool addArrayOfSingleValueTo(IN const std::string &arrayKeyPath, IN const std::vector<T> &values, OUT JsonTree &jt)
        {
            JsonTree arrayProperty;

            try
            {
                newArray(values, arrayProperty);
                jt.add_child(arrayKeyPath, arrayProperty);
            }

            catch (boost::exception& e)
            {
                COM_LOG_ERROR << "Boost exception when adding child to " << arrayKeyPath;
                COM_LOG_ERROR << boost::diagnostic_information(e);
                return false;
            }

            catch (std::exception &e)
            {
                COM_LOG_ERROR << "STD exception when adding child to " << arrayKeyPath;
                COM_LOG_ERROR << e.what();
                return false;
            }

            catch (...)
            {
                COM_LOG_ERROR << "Unknow exception when adding child to " << arrayKeyPath;
                return false;
            }

            return true;
        }

        template<typename T>
        bool addArrayOfSingleValueTo(IN const std::string &arrayKeyPath, IN const std::vector<std::vector<T>> &objects, OUT JsonTree &jt)
        {
            JsonTree l2Node;
            JsonTree l3Node;

            try
            {
                for (auto const & vecItem : objects)
                {
                    newArray(vecItem, l3Node);
                    l2Node.push_back(std::make_pair("", l3Node));
                }

                jtree_.add_child(arrayKeyPath, l2Node);
            }

            catch (boost::exception& e)
            {
                COM_LOG_ERROR << "Boost exception when adding child to " << arrayKeyPath;
                COM_LOG_ERROR << boost::diagnostic_information(e);
                return false;
            }

            catch (std::exception &e)
            {
                COM_LOG_ERROR << "STD exception when adding child to " << arrayKeyPath;
                COM_LOG_ERROR << e.what();
                return false;
            }

            catch (...)
            {
                COM_LOG_ERROR << "Unknow exception when adding child to " << arrayKeyPath;
                return false;
            }

            return true;
        }

        template<typename T>
        bool addArrayOfSingleValue(IN const std::string &arrayKeyPath, IN const std::vector<T> &values)
        {
            return addArrayOfSingleValueTo(arrayKeyPath, values, jtree_);
        }

        template<typename T>
        bool addArrayOfSingleValue(IN const std::string &arrayKeyPath, IN const std::vector<std::vector<T>> &values)
        {
            return addArrayOfSingleValueTo(arrayKeyPath, values, jtree_);
        }

        /*
         * when objElements is empty, the output json file will not include a pair of square bracket
         * around the empty quotes "", that will lead to the parse failure on FWS side.
         */
        template<typename T>
        bool addArrayOfObject(IN const std::string &arrayKeyPath,
                                std::vector<std::map<std::string, T>> &objElements)
        {
            JsonTree objArray;

            if (objElements.empty())
            {
                objElements.emplace_back();
            }

            auto it = objElements.begin();

            try
            {
                for (; it != objElements.end(); ++it)
                {
                    JsonTree obj;

                    newObject(*it, obj);
                    objArray.push_back(std::make_pair("", obj));
                }

                jtree_.add_child(arrayKeyPath, objArray);
            }

            catch (boost::exception& e)
            {
                COM_LOG_ERROR << "Boost exception when addArrayOfObject to " << arrayKeyPath;
                COM_LOG_ERROR << boost::diagnostic_information(e);
                return false;
            }

            catch (std::exception &e)
            {
                COM_LOG_ERROR << "STD exception when addArrayOfObject to " << arrayKeyPath;
                COM_LOG_ERROR << e.what();
                return false;
            }

            catch (...)
            {
                COM_LOG_ERROR << "Unknow exception when addArrayOfObject to " << arrayKeyPath;
                return false;
            }

            return true;
        }

        bool addArrayOfNodeTo(IN const std::string &arrayKeyPath, IN const std::vector<JsonTree> &objElements, OUT JsonTree &JTto);
        bool addArrayOfNode(IN const std::string &arrayKeyPath, IN const std::vector<JsonTree> &objElements);
        bool addNodeToArray(IN const std::string &arrayKeyPath, IN JsonTree &node);

        /*save to file*/
        bool save(IN const std::string &filePath, bool pretty = false);
        bool save(OUT std::ostringstream &ss, bool pretty = false);

        /*other utility functions*/
        void print();
        void out2Log();
        bool isSingleValue(IN const std::string &keyPath);
        bool isSingleValue(IN const JsonTree &jt, IN const std::string &keyPath);
        bool isObject(IN const std::string &keyPath);
        bool isArray(IN const std::string &keyPath);
        bool isArray(IN const JsonTree &jt, IN const std::string &keyPath);
        int32_t getArrayCount(IN const std::string &keyPath);
        int32_t getArrayCountOfNode(IN const JsonTree &jt, IN const std::string &keyPath);

    public:
        template<typename valueType>
        bool newObject(IN const std::map<std::string, valueType> &properties, INOUT JsonTree &newObject)
        {
            newObject.clear();

            auto it = properties.begin();

            try
            {
                for (; it != properties.end(); ++it)
                {
                    newObject.add(it->first, it->second);
                }
            }

            catch (boost::exception& e)
            {
                COM_LOG_ERROR << "Boost exception when adding object!";
                COM_LOG_ERROR << boost::diagnostic_information(e);
                return false;
            }

            catch (std::exception &e)
            {
                COM_LOG_ERROR << "STD exception when adding object!";
                COM_LOG_ERROR << e.what();
                return false;
            }

            catch (...)
            {
                COM_LOG_ERROR << "Unknow exception whe adding object!";
                return false;
            }

            return true;
        }

        template<typename T>
        bool newArray(IN const std::vector<T> &elements, INOUT JsonTree &newObject)
        {
            newObject.clear();

            auto it = elements.begin();

            try
            {
                for (; it != elements.end(); ++it)
                {
                    JsonTree value;

                    value.put_value(*it);
                    newObject.push_back(std::make_pair("", value));
                }
            }

            catch (boost::exception& e)
            {
                COM_LOG_ERROR << "Failed to call newArray due to boost exception!";
                COM_LOG_ERROR << boost::diagnostic_information(e);

                return false;
            }

            catch (std::exception &e)
            {
                COM_LOG_ERROR << "Failed to call newArray due to STD exception!";
                COM_LOG_ERROR << e.what();
                return false;
            }

            catch (...)
            {
                COM_LOG_ERROR << "Failed to call newArray due to unknow exception!";
                return false;
            }

            return true;
        }

        bool getElementofArray(IN const std::string &keyPath, IN int32_t index, OUT JsonTree &elementT);
        bool getElementofArray(IN const JsonTree &arrayTree, IN int32_t index, OUT JsonTree &elementT);

    private:
        JsonTree jtree_;
    };


}

#endif
