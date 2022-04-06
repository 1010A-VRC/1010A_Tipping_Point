/**
 * @file opcontrol.cpp
 * @author Liam Teale
 * @brief File containing all the functions relating to operator control
 * @version 0.1
 * @date 2022-01-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "main.h"
#include "robot_config.hpp"
#include "opcontrol.hpp"

using namespace pros;



std::uint8_t move_left_drive(std::int32_t voltage)
{
    /* move motors */
    l1.move(voltage);
    l2.move(voltage);
    l3.move(voltage);
}




/**
 * @brief function that calculates the value of an exponent
 * 
 * @param base 
 * @param coefficient 
 * @param exponent
 * 
 * @return double 
 *
 */
double user_control::calculate_expo(double base, double coefficient, double exponent)
{
    /* return the answer */
    return std::pow(base, exponent) * base;
}



/**
 * @brief Operator control code
 *
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 *
 */
void user_control::op_control() 
{
    /* create variables */
    std::uint32_t loop_start_time; /**< time of the program at the start of the loop */
    std::int32_t l_joystick_y; /**< left joystick Y value  */
    std::int32_t r_joystick_x; /**< right joystick Y value */
    
    /* main loop */
    while (true) {
        /* get the time */
        loop_start_time = pros::millis();

        /* update joystick values */
        

        /* delay to allow RTOS to run other tasks */
        Task::delay_until(&loop_start_time, 5);
    }

}
