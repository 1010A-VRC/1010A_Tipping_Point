/**
 * @file opcontrol.hpp
 * @author Liam Teale
 * @brief header file for all the functions relating to operator control
 * @version 0.1
 * @date 2022-01-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

/**
 * @brief user control class
 * 
 */
class user_control {

    public:
        void op_control();

    private:
        std::uint8_t move_left_drive(std::int32_t voltage);
        std::uint8_t move_right_drive(std::int32_t voltage);
        double calculate_expo(double base, double coefficient, double exponent);



};

