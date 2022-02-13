/**
 * @file fileIO.cpp
 * @author Liam Teale
 * @brief file containing functions for file I/o
 * @version 0.1
 * @date 2022-01-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "main.h"
#include "dataManagement/fileIO.hpp"

using namespace std;


/**
 * @brief Construct a new sd card::sd card object
 * 
 * @param path the path of the file
 * @param mode the mode of the file, such as r+
 */
sd_card::sd_card(const char* path, const char* mode) {
  /** open the file */
  sd_card::file = fopen(path, mode);
}


/**
 * @brief function that returns the lengths of a string
 * 
 * @param s the string that is to be measured
 * @return int 
 */
int sd_card::getSize(char* s) {
    char * t;    
    for (t = s; *t != '\0'; t++)
        ;
    return t - s;
}


/**
 * @brief Function that reads a line from the sd card
 * 
 * @param buffer the maximum number of characters to be read
 * @return std::string 
 */
std::string sd_card::read_line(int buffer) {
    char buff[buffer]; /**< create a character array                     */
    fgets(buff, buffer, sd_card::file); /**< get input from the file     */
    return std::string(buff);
}


/**
 * @brief Function that writes a string to a file
 * 
 * @param output 
 */
void sd_card::write_string(std::string output) {
    const char* out; /**< create char array                          */
    out = output.c_str(); /**< convert the parameter to a char array */
    fputs(out, sd_card::file); /**< write the line to the file       */
}


/**
 * @brief function that reads all elements from the file
 * 
 * @param buffer the maximum number of characters in a line
 * @param delimiter the string or character seperating the data in the file
 * @return std::vector<std::string> 
 */
std::vector<std::string> sd_card::read_elements(int buffer, std::string delimiter) {

    /** define objects                                */
    string token; /**< define string                  */
    char buff[buffer]; /**< input character array     */
    vector<string> output;

    /** read the next line in the file */
    fgets(buff, buffer, file);

    /** convert the type of the read line */
    std::string s = string(buff);

    /** read the line of the file, and add any data to a vector */
    size_t pos = 0;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        output.push_back(token);
        s.erase(0, pos + delimiter.length());
    }

    s.pop_back(); /**< remove the endline character from the read line */
    output.push_back(s); /**< add the last piece of data to the vector */

    return output; 
}


