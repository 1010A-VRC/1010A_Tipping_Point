#pragma once


/**
 * @brief Function that moves the left side of the drivetrain by controlling the voltage
 * 
 * @param speed how fast to move the left side of the drivetrain, from -127 to 127
 *
 */
void moveLeftDrivetrain(double speed);


/**
 * @brief Function that moves the right side of the drivetrain by controlling the voltage
 * 
 * @param speed how fast to move the right side of the drivetrain, from -127 to 127
 *
 */
void moveRightDrivetrain(double speed);


/**
 * @brief Function that stops all motors on the drivetrain
 * 
 */
void stopDrivetrain();


/**
 * @brief Function that makes the robot go in a straight line towards a point
 * 
 * @param goalX the X position of the goal
 * @param goalY the Y position of the goal
 * @param kJ jerk parameter, controls the acceleration of the robot
 * @param kP proportional parameter
 * @param kI integral parameter
 * @param kD derivative parameter
 * @param maxTime timeout
 *
 */
void forwardJPIDTrack(double goalX, double goalY, double kJ, double kP, double kI, double kD, double maxTime);


/**
 * @brief Function that makes the robot go in a straight line towards a point
 * 
 * @param goalX the X position of the goal
 * @param goalY the Y position of the goal
 * @param kJ jerk parameter, controls the acceleration of the robot
 * @param kP proportional parameter
 * @param kI integral parameter
 * @param kD derivative parameter
 * @param maxTime timeout
 *
 */
void backwardJPIDTrack(double goalX, double goalY, double kJ, double kP, double kI, double kD, double maxTime);


/**
 * @brief Function that goes forward by using the front distance sensor
 * 
 * @param goal the goal value of the distance sensor, how far away the tracked object is from the distance sensor
 * @param expectedDistance the expected distance from the tracked object and the distance sensor at the start of the movement
 * @param clampOffset how long the function needs to use expectedDistance as error at the start, to allow for the front clamp to go up
 * @param kJ jerk parameter, controls the acceleration of the robot
 * @param kP proportional parameter
 * @param kI integral parameter
 * @param kD derivative parameter
 * @param maxTime timeout
 *
 */
void forwardJPIDfrontDistance(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime);


/**
 * @brief PID using the back distance sensor to move the robot forwards 
 * 
 * @param goal the target distance between the tracked object and the distance sensor
 * @param expectedDistance the expected distance from the tracked object and the distance sensor at the start of the movement
 * @param clampOffset how long the function needs to use expectedDistance as error at the start, to allow for the back clamp to go up
 * @param kJ jerk parameter, controls the acceleration of the robot
 * @param kP proportional parameter
 * @param kI integral parameter
 * @param kD derivative parameter
 * @param maxTime timeout
 *
 */
void forwardJPIDbackDistance(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime);


/**
 * @brief function that makes the robot move backwards using data from the back distance sensor
 * 
 * @param goal the target distance between the tracked object and the distance sensor
 * @param expectedDistance the expected distance from the tracked object and the distance sensor at the start of the movement
 * @param clampOffset how long the function needs to use expectedDistance as error at the start, to allow for the back clamp to go up
 * @param kJ jerk parameter, controls the acceleration of the robot
 * @param kP proportional parameter
 * @param kI integral parameter
 * @param kD derivative parameter
 * @param maxTime timeout
 *
 */
void backwardJPIDbackDistance(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime);


/**
 * @brief function that makes the robot move backwards using data from the back distance sensor
 * 
 * @param goal the target distance between the tracked object and the distance sensor
 * @param expectedDistance the expected distance from the tracked object and the distance sensor at the start of the movement
 * @param clampOffset how long the function needs to use expectedDistance as error at the start, to allow for the back clamp to go up
 * @param kJ jerk parameter, controls the acceleration of the robot
 * @param kP proportional parameter
 * @param kI integral parameter
 * @param kD derivative parameter
 * @param maxTime timeout
 *
 */
void backwardJPIDbackDistance2(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime);


/**
 * @brief PID that turns the robot
 * 
 * @param goal the desired heading the robot should face
 * @param kJ parameter used to control the acceleration of the robot
 * @param kP proportional 
 * @param kI integral
 * @param kD derivative
 * @param maxTime timeout
 *
 */
void turnJPID(double goal, double kJ, double kP, double kI, double kD, double maxTime);


/**
 * @brief PID that turns the robot to face an orientation
 * 
 * @param goal the desired heading the robot should face
 * @param kJ parameter used to control the acceleration of the robot
 * @param kP proportional 
 * @param kI integral
 * @param kD derivative
 * @param maxTime timeout
 *
 */
void turnJPID2(double goal, double kJ, double kP, double kI, double kD, double maxTime);


/**
 * @brief back vision align function
 * 
 * @param sigID signature ID number
 * @param sig pointer to the signature
 * @param turnKP proportional
 * @param turnKI integral
 * @param turnKD derivative
 * @param maxTime timeout
 *
 */
void back_vision_align(int sigID, pros::vision_signature_s_t* sig, double turnKP, double turnKI, double turnKD, double maxTime);


/**
 * @brief function that moves the robot a number of inches relative to the robots position at the start of the function
 * 
 * @param goal how far the robot should move
 * @param kJ how fast the robot can accelerate
 * @param kP proportional
 * @param kI integral
 * @param kD derivative
 * @param maxTime timeout
 *
 */
void basicForwardJPID(double goal, double kJ, double kP, double kI, double kD, double maxTime);
