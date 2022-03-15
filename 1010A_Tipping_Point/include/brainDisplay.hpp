/**
 * @file brainDisplay.hpp
 * @author Liam Teale
 * @brief Header file containing definitions for functions and variables for the brain screen
 * @version 0.1
 * @date 2022-01-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once


/**
 * @brief class containing functions used on the brain display
 * 
 */
class brain_screen {

    public:
        static void legacy_brain_display();
        static void brain_display();

    private:
        static double average_temperature(pros::Motor *a, pros::Motor *b, pros::Motor *c);
        static double average_wattage(pros::Motor *a, pros::Motor *b, pros::Motor *c);


};
