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
        static void drivetrain_control();       /**< Method controlling the drivetrain              */
        static void front_lift_control();       /**< Method controlling the front lift              */
        static void front_clamp_control();      /**< Method controlling the front clamp             */
        static void back_clamp_control();       /**< Method controlling the back clamp              */
        static void conveyor_control();         /**< Method controllinf the conveyor                */

    private:
        static void front_lift_stop();          /**< Method using a PID */


};

