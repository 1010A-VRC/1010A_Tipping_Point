/**
 * @file robot_config.cpp
 * @author Liam Teale
 * @brief File containing constructors for all electronic devices
 * @version 0.1
 * @date 2022-01-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "main.h"
#include "pros/vision.h"
#include "robot_config.hpp"
#include "autoFunctions/odometry.hpp"

using namespace pros;

/**
 * @brief Controllers
 * 
 */
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Controller* partner;
pros::Controller partnerController(pros::E_CONTROLLER_PARTNER);

/**
 * @brief V5 motors
 *
 * 6 motors are used in the drivetrain
 * 1 motor is used for a 4 bar
 * 1 motor is used for a conveyor
 * 
 */
pros::Motor l1 (18, pros::E_MOTOR_GEARSET_18, true); 
pros::Motor l2 (14, pros::E_MOTOR_GEARSET_18, false); 
pros::Motor l3 (13, pros::E_MOTOR_GEARSET_18, true);  
pros::Motor r1 (20, pros::E_MOTOR_GEARSET_18, false); 
pros::Motor r2 (16, pros::E_MOTOR_GEARSET_18, true); 
pros::Motor r3 (21, pros::E_MOTOR_GEARSET_18, false); 
pros::Motor lift (3, pros::E_MOTOR_GEARSET_36, false);
pros::Motor conveyor (2, pros::E_MOTOR_GEARSET_06, false);

/**
 * @brief V5 sensors
 *
 * 2 inertial sensors
 * 1 vision sensor
 * 1 rotation sensor
 * 
 */
pros::IMU imu1 (11);
pros::IMU imu2 (12);
pros::Vision frontVision (1);
pros::Distance frontDistance (6);
pros::Vision backVision (8);
pros::Distance backDistance (19);
pros::Rotation frontLiftRotation (4); 

/**
 * @brief Front vision sensor signatures
 * 
 * Contains signature objects for the front vision sensor
 * The vision sensor is highly sensitive to changes in lighting 
 * Signatures for different locations are highly recommended
 *
 */
pros::vision_signature_s_t front_red_mogo_alderfeild = pros::Vision::signature_from_utility (3, 6197, 8651, 7424, -873, -313, -594, 3, 0); /**< red mogo for the front vision sensor at the Alderfeild field        */
pros::vision_signature_s_t front_blue_mogo_alderfeild = pros::Vision::signature_from_utility (2, -2265, -1353, -1810, 6479, 9495, 7986, 3, 0);; /**< blue mogo for the front vision sensor at the Alderfeild field    */
pros::vision_signature_s_t front_yellow_mogo_alderfeild = pros::Vision::signature_from_utility(3, -1, 2079, 1038, -3665, -2527, -3096, 2.3, 0);

/**
 * @brief Back vision sensor signatures
 * 
 * Contains signature objects for the back vision sensor
 * The vision sensor is highly sensitive to changes in lighting 
 * Signatures for different locations are highly recommended
 *
 */
pros::vision_signature_s_t back_red_mogo_alderfeild = pros::Vision::signature_from_utility (1, 4445, 10685, 7566, -1781, 133, -824, 2.1, 0); /**< red mogo for the back vision sensor at the Alderfeild field       */
pros::vision_signature_s_t back_blue_mogo_alderfeild = pros::Vision::signature_from_utility (2, -3687, -2001, -2844, 7461, 12243, 9852, 2.6, 0); /**< blue mogo for the back vision sensor at the Alderfeild field  */
pros::vision_signature_s_t back_yellow_mogo_alderfeild = pros::Vision::signature_from_utility (3, 1035, 1857, 1446, -4025, -2953, -3490, 3, 0); /**< yellow mogo for the back vision sensor at the Alderfeild field */


/**
 * @brief front vision sensor sign
 * 
 */

/**
 * @brief 3-wire sensors
 * 
 * 1 vertical tracking wheel
 * 1 horizontal tracking wheel
 * 
 */
pros::ADIEncoder sideTrackingWheel ('E', 'F'); 
pros::ADIEncoder centerTrackingWheel ('C', 'D', true);

/**
 * @brief 3-wire outputs
 * 
 * 1 piston for the front clamp
 * 1 piston for the back clamp
 * 
 */
pros::ADIDigitalOut frontClamp ('A', false); 
pros::ADIDigitalOut backClamp ('B', false);
