/**
 * @file robot_config.hpp
 * @author Liam Teale
 * @brief 
 * @version 0.1
 * @date 2022-01-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

/**
 * @brief Enumerator containing the positions for the front lift
 * 
 */
typedef enum frontLiftPositions {
  E_FRONT_LIFT_HOME = 250,
  E_FRONT_LIFT_CONVEYOR = 1400,
  E_FRONT_LIFT_RAMP = 2500,
  E_FRONT_LIFT_RAMP_DOWN = 2337,
  E_FRONT_LIFT_MAX = 11700
} frontLiftPositionEnum;

/**
 * @brief 
 * Controllers
 * 
 */
extern pros::Controller master;
extern pros::Controller* partner;
extern pros::Controller partnerController;

/**
 * @brief
 * V5 Motors
 * 6 motors are used in the drivetrain
 * 1 motor is used for a 4 bar
 * 1 motor is used for a conveyor
 */
extern pros::Motor l1;
extern pros::Motor l2;
extern pros::Motor l3; 
extern pros::Motor r1; 
extern pros::Motor r2; 
extern pros::Motor r3; 
extern pros::Motor lift; 
extern pros::Motor conveyor;

/**
 * @brief
 * V5 sensors
 * 2 inertial sensors
 * 1 vision sensor
 * 1 rotation sensor
 */
extern pros::IMU imu1;
extern pros::IMU imu2;
extern pros::Vision frontVision;
extern pros::Distance frontDistance;
extern pros::Vision backVision;
extern pros::Distance backDistance;
extern pros::Rotation frontLiftRotation;

/**
 * @brief Front vision sensor signatures
 * 
 * Contains signature objects for the front vision sensor
 * The vision sensor is highly sensitive to changes in lighting 
 * Signatures for different locations are highly recommended
 *
 */
extern pros::vision_signature_s_t front_red_mogo_alderfeild; /**< red mogo for the front vision sensor at the Alderfeild field       */
extern pros::vision_signature_s_t front_blue_mogo_alderfeild; /**< blue mogo for the front vision sensor at the Alderfeild field     */
extern pros::vision_signature_s_t front_yellow_mogo_alderfeild; /**< yellow mogo for the front vision sensor at the Alderfeild field */

extern pros::vision_signature_s_t f_y_mogo_ald_up;

extern pros::vision_signature_s_t f_r_mogo_sit; /**< red mogo for the front vision sensor at the Situation field    */
extern pros::vision_signature_s_t f_b_mogo_sit; /**< blue mogo for the front vision sensor at the Situation field   */
extern pros::vision_signature_s_t f_y_mogo_sit; /**< yellow mogo for the front vision sensor at the Situation field */

extern pros::vision_signature_s_t f_y_mogo_board;

/**
 * @brief Back vision sensor signatures
 * 
 * Contains signature objects for the back vision sensor
 * The vision sensor is highly sensitive to changes in lighting 
 * Signatures for different locations are highly recommended
 *
 */
extern pros::vision_signature_s_t back_red_mogo_alderfeild; /**< red mogo for the back vision sensor at the Alderfeild field       */
extern pros::vision_signature_s_t back_blue_mogo_alderfeild; /**< blue mogo for the back vision sensor at the Alderfeild field     */
extern pros::vision_signature_s_t back_yellow_mogo_alderfeild; /**< yellow mogo for the back vision sensor at the Alderfeild field */

/**
 * @brief 
 * 3-wire sensors
 * 1 vertical tracking wheel
 * 1 horizontal tracking wheel
 */
extern pros::ADIEncoder sideTrackingWheel; 
extern pros::ADIEncoder centerTrackingWheel;

/**
 * @brief 
 * 3-wire outputs
 * 1 piston for the front clamp
 * 1 piston for the back clamp
 */
 extern pros::ADIDigitalOut frontClamp; 
 extern pros::ADIDigitalOut backClamp;