/**
 * @file main.cpp
 * @author Liam Teale
 * @brief Main file
 * @version 0.1
 * @date 2022-01-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "main.h"
#include "robot_config.hpp"
#include "dataManagement/fileIO.hpp"
#include "brainDisplay.hpp"
#include "autoFunctions/odometry.hpp"
#include "opcontrol.hpp"
#include "autoFunctions/basicOdomPID.hpp"
#include "autoFunctions/subsystems.hpp"
#include "autos.hpp"


/**
 * @brief Create instances of widely used classes
 * 
 */
odometry odom(&sideTrackingWheel, false, 2.75, 1.5, &centerTrackingWheel, true, 2.75, 1.5);


/**
 * @brief Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 *
 */
void initialize () 
{
	pros::lcd::initialize(); /**< initialize LLEMU  */

	imu1.reset();
	imu2.reset();

	odom.start_tracking(); /**< initialize odometry */

	/** set all drivetrain motors to hold, in order to make accurate turns and not to slide off the ramp */
	//l1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	//l2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	//l3.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	//r1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	//r2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	//r3.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	/** create a task for the brain screen display */
	pros::Task brainDisplay(brain_screen::legacy_brain_display);

}


/**
 * @brief Runs disablemet code
 *
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 *
 */
void disabled () 
{
	
}


/**
 * @brief Competition initialization code
 *
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 *
 */
void competition_initialize () 
{

}


/**
 * @brief Autonomous code
 *
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 *
 */
void autonomous () 
{
	right3G(&front_red_mogo_alderfeild, 1, &front_red_mogo_alderfeild, 1);

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
void opcontrol () 
{
	/** create tasks for drivetrain control */
	pros::Task drivetrainControl(user_control::drivetrain_control); /**< task controlling the drivetrain 						*/
	pros::Task frontLiftControl(user_control::front_lift_control); /**< task controlling the front lift 						*/
	pros::Task frontClampControl(user_control::front_clamp_control); /**< task controlling the front clamp 						*/
	pros::Task backClampControl(user_control::back_clamp_control); /**< task controlling the back clamp							*/
	pros::Task conveyorControl(user_control::conveyor_control); /**< task controlling the conveyor 								*/
	pros::Task drivetoggle(user_control::driveToggle);
	
}
