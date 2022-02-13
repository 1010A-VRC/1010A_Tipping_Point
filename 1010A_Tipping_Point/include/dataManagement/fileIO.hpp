/**
 * @file fileIO.hpp
 * @author Liam Teale
 * @brief Header file containing definitions for functions for file I/O
 * @version 0.1
 * @date 2022-01-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once


/**
 * @brief sd_card class. Used to read and write to files
 * 
 */
class sd_card {

  public:
    sd_card(const char* path, const char* mode);
    std::string read_line(int buffer);
    void write_string(std::string output);
    std::vector<std::string> read_elements(int buffer, std::string delimiter);

    
  private:
    FILE* file;

    int getSize(char* s);


};