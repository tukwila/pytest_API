/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 ******************************************************************************
 * @file   option_parser.h
 * @brief  Declaration of option_parser
 *******************************************************************************
 */
/**********************************************************************************
 *
 * company         : ygomi 
 * author          : chenxiang li
 * description     : option parser 
 * create time     : 2016.4.7 
 * ********************************************************************************/



#ifndef OPTION_PARSER_H
#define OPTION_PARSER_H
#include <string>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <stdint.h>

#include <getopt.h>
typedef class OptionPriv_ OptionPriv;

class OptionParser
{
    public:
        ~OptionParser();
        static OptionParser *getInstance();

        void add_option(const std::string &cmd, char short_cmd,const std::string &dest, const std::string &def_val, const std::string &help,int has_arg = required_argument);
        void parse_args(int argc,char **argv);
        std::string get_value(const std::string &dest);
        void dump_options();
        void help_options(const std::string &);
        bool bset(const std::string &dest);
    private:
        //func
        OptionParser(const OptionParser&);
        OptionParser& operator = (const OptionParser&); 
        OptionParser();
        std::string build_short_args();
        void build_long_options(struct option ** long_options);

    private:
        //value
        OptionPriv *opPriv_;
};

#endif
