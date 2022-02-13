/**
 * @file brainDisplay.cpp
 * @author Liam Teale
 * @brief File containing functions for the brain screen display
 * @version 0.1
 * @date 2022-01-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "main.h"
#include "robot_config.hpp"
#include "autoFunctions/odometry.hpp"
#include "brainDisplay.hpp"


/**
 * @brief function that returns the average temperature of 3 motors
 * 
 * @param a motor a 
 * @param b motor b
 * @param c motor c
 * @return double 
 */
double brain_screen::average_temperature(pros::Motor *a, pros::Motor *b, pros::Motor *c) {
    return (a->get_power() + b->get_power() + c->get_power())/3;
}


/**
 * @brief functions using LLEMU, the legacy LED emulator
 * 
 */
void brain_screen::legacy_brain_display() 
{

    std::string a, b, c;
    /** loop forever */

    

    while (true) {

        pros::lcd::print(0, "X: %f", odom.get_x());
        pros::lcd::print(1, "Y: %f", odom.get_y());
        pros::lcd::print(2, "H: %f", odom.get_heading(false));
        pros::lcd::print(3, "Left side Temperature: %f", brain_screen::average_temperature(&l1, &l2, &l3)); /**< print the left drivetrain motor temperatures   */
        pros::lcd::print(4, "Right Side Temperature: %f", brain_screen::average_temperature(&r1, &r2, &r3)); /**< print the right drivetrain motor temperatures */
        pros::lcd::print(5, "Conveyor Temperature: %f", conveyor.get_temperature()); /**< print the temperature of the conveyor motor             */
        pros::lcd::print(6, "Lift temperature: %f", lift.get_temperature()); /**< print the temperature of the lift motor                         */
        pros::lcd::print(7, "Front distance sensor: %d", frontDistance.get());

        /** delay to prevent RTOS freezing */
        pros::delay(15);
    }
}