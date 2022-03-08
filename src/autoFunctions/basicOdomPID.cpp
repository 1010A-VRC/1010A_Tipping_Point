#include "main.h"
#include "pros/vision.h"
#include "robot_config.hpp"
#include "autoFunctions/odometry.hpp"
#include "autoFunctions/basicOdomPID.hpp"

using namespace pros;


/**
 * @brief Function that moves the left side of the drivetrain by controlling the voltage
 * 
 * @param speed how fast to move the left side of the drivetrain, from -127 to 127
 *
 */
void moveLeftDrivetrain(double speed) 
{
    l1.move(speed);
    l2.move(speed);
    l3.move(speed);
}


/**
 * @brief Function that moves the right side of the drivetrain by controlling the voltage
 * 
 * @param speed how fast to move the right side of the drivetrain, from -127 to 127
 *
 */
void moveRightDrivetrain(double speed) 
{
    r1.move(speed);
    r2.move(speed);
    r3.move(speed);
}


/**
 * @brief Function that stops all motors on the drivetrain
 * 
 */
void stopDrivetrain() {
    l1.move(0);
    l2.move(0);
    l3.move(0);
    r1.move(0);
    r2.move(0);
    r3.move(0);
}


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
void forwardJPIDTrack(double goalX, double goalY, double kJ, double kP, double kI, double kD, double maxTime) 
{
    double newGoal = sideTrackingWheel.get_value() + (std::sqrt(std::pow(goalX - odom.get_x(), 2) + std::pow(goalY - odom.get_y(), 2)) / (M_PI*2.75))*360; /**< get the distance between the robot and the target point */
    double error = newGoal - sideTrackingWheel.get_value(); /**< calculate the error                                                                                                                                    */
    double prevError = error; /**< the value of error at last cycle                                                                                                                                                     */
    double derivative = error - prevError; /**< the change in error since last cycle                                                                                                                                    */
    double totalError = 0; /**< the total error since the start of the function                                                                                                                                         */
    double motorPower = 0; /**< the power to be sent to the drivetrain                                                                                                                                                  */
    double prevMotorPower = 0; /**< the value of motorPower at last cycle                                                                                                                                               */
    double slew = kJ; /**< the maximum change in velocity                                                                                                                                                               */

    /** main loop */
    for (int time = 0; time < maxTime; time+=10) {

        /** calculate variables */
        error = newGoal - sideTrackingWheel.get_value(); /**< calculate error                   */
        derivative = error - prevError; /**< calculate derivative                               */
        prevError = error; /**< update prevError                                                */
        totalError += error; /**< calculate error                                               */
        slew += kJ; /**< update acceleration                                                    */
        motorPower = error * kP + totalError * kI + derivative * kD; /**< calculate motor power */

        /** break out of the loop if necessary */
        if (fabs(error) < 10) {
            break;
        }

        /** slew */
        if (motorPower - prevMotorPower > slew) {
            motorPower = prevMotorPower + slew;
        }
        prevMotorPower = motorPower; /**< update the previous motor power */

        /** spin the motors */
        moveLeftDrivetrain(motorPower);
        moveRightDrivetrain(motorPower);
        
        /** prevent function hogging resources */
        pros::delay(10);
    }
    /** stop the drivetrain in case it is not already */
    stopDrivetrain();
}


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
void backwardJPIDTrack(double goalX, double goalY, double kJ, double kP, double kI, double kD, double maxTime) 
{
    double newGoal = sideTrackingWheel.get_value() - (std::sqrt(std::pow(goalX - odom.get_x(), 2) + std::pow(goalY - odom.get_y(), 2)) / (M_PI*2.75))*360; /**< get the distance between the robot and the target point */
    double error = newGoal - sideTrackingWheel.get_value(); /**< calculate the error                                                                                                                                    */
    double prevError = error; /**< the value of error at last cycle                                                                                                                                                     */
    double derivative = error - prevError; /**< the change in error since last cycle                                                                                                                                    */
    double totalError = 0; /**< the total error since the start of the function                                                                                                                                         */
    double motorPower = 0; /**< the power to be sent to the drivetrain                                                                                                                                                  */
    double prevMotorPower = 0; /**< the value of motorPower at last cycle                                                                                                                                               */
    double slew = kJ; /**< the maximum change in velocity                                                                                                                                                               */

    /** main loop */
    for (int time = 0; time < maxTime; time+=10) {

        /** calculate variables */
        error = newGoal - sideTrackingWheel.get_value(); /**< calculate error                   */
        derivative = error - prevError; /**< calculate derivative                               */
        prevError = error; /**< update prevError                                                */
        totalError += error; /**< calculate error                                               */
        slew += kJ; /**< the maximum change in velocity                                         */
        motorPower = error * kP + totalError * kI + derivative * kD; /**< calculate motor power */

        /** break out of the loop if necessary */
        if (fabs(error) < 10) {
            break;
        }

        /** slew */
        if (motorPower - prevMotorPower < -slew) {
            motorPower = prevMotorPower - slew;
        }
        prevMotorPower = motorPower; /**< update the previous motor power */

        /** spin the motors */
        moveLeftDrivetrain(motorPower);
        moveRightDrivetrain(motorPower);
        
        /** prevent function hogging resources */
        pros::delay(10);
    }
    /** stop the drivetrain in case it is not already */
    stopDrivetrain();
}


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
void forwardJPIDfrontDistance(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime) 
{
    double error = 0; /**< calculate the error                                       */
    double prevError = expectedDistance; /**< the value of error at last cycle       */
    double derivative = error - prevError; /**< the change in error since last cycle */
    double totalError = 0; /**< the total error since the start of the function      */
    double motorPower = 0; /**< the power to be sent to the drivetrain               */
    double prevMotorPower = 0; /**< the value of motorPower at last cycle            */
    double slew = kJ; /**< the maximum change in velocity                            */

    /** main loop */
    for (double time = 0; time < maxTime; time+=10) {

        /** calculate error, if statement to decide whether to use expectedDistance */
        if (time < clampOffset) {
            error = -expectedDistance;
        } else {
            if (frontDistance.get() < 5) {
                error = prevError; /**< calculate error using expectedDistance as position */
            } else {
                error = goal - frontDistance.get(); /**< calculate error normally */
            }
        }

        derivative = error - prevError; /**< calculate derivative                               */
        prevError = error; /**< update prevError                                                */
        totalError += error; /**< calculate error                                               */
        slew += kJ; /**< update acceleration                                                    */
        motorPower = error * kP + totalError * kI + derivative * kD; /**< calculate motor power */

        // break out of the loop if necessary 
        if (error < 40) {
            break;
        }

        // slew
        if (motorPower - prevMotorPower < -slew) {
            motorPower = prevMotorPower - slew;
        }
        prevMotorPower = motorPower;

        // spin the motors
        moveLeftDrivetrain(-motorPower);
        moveRightDrivetrain(-motorPower);

        printf("error: %f \r\n", error);
        
        pros::delay(10);
    }
    stopDrivetrain();
}


// forward PID using front distance sensor
void specialForwardJPIDfrontDistance(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime) 
{
    double error = 0; // calculate the error
    double prevError = expectedDistance; // the value of error at last cycle
    double derivative = error - prevError; // the change in error since last cycle
    double totalError = 0; // the total error since the start of the function
    double motorPower = 0; // the power to be sent to the drivetrain
    double prevMotorPower = 0; // the value of motorPower at last cycle
    double slew = kJ; // the maximum change in velocity

    double totalPower = 0;

    for (double time = 0; time < maxTime; time+=10) {

        // calculate stuff
        if (time < clampOffset) {
            error = -expectedDistance;
        } else {
            if (frontDistance.get() < 5) {
                error = prevError;
            } else {
                error = goal - frontDistance.get(); // calculate error
            }
        }

        totalPower += (l1.get_actual_velocity()+l2.get_actual_velocity())/2;
        derivative = error - prevError; // calculate derivative
        prevError = error; // update prevError
        totalError += error; // calculate error
        slew += kJ;
        motorPower = error * kP + totalError * kI + derivative * kD; // calculate motor power

        // break out of the loop if necessary
        if (fabs(error) < 40 && time > 880) {
            totalPower /= time/10;
            pros::lcd::print(7, "average speed: %f", totalPower);
            break;
        }

        /** slow down the acceleration of the robot if necessary */
        if (motorPower - prevMotorPower < -slew) {
            motorPower = prevMotorPower - slew;
        }
        prevMotorPower = motorPower; /**< update the previous motor power */

        /** spin the motors */
        moveLeftDrivetrain(-motorPower);
        moveRightDrivetrain(-motorPower);
        
        /** delay to prevent all resources from being used */
        pros::delay(10);
    }
    /** stop all motors on the drivetrain */
    stopDrivetrain();
}


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
void forwardJPIDbackDistance(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime) 
{
    double error = goal - backDistance.get(); /**< calculate the error               */
    double prevError = error; /**< the value of error at last cycle                  */
    double derivative = error - prevError; /**< the change in error since last cycle */
    double totalError = 0; /**< the total error since the start of the function      */
    double motorPower = 0; /**< the power to be sent to the drivetrain               */
    double prevMotorPower = 0; /**< the value of motorPower at last cycle            */
    double slew = kJ; /**< the maximum change in velocity                            */

    /** main loop */
    for (int time = 0; time < maxTime; time+=10) {

        /** calculate error, if statement to decide whether to use expectedDistance */
        if (time < clampOffset) {
            error = expectedDistance; /**< calculate error using expectedDistance as position */
        } else {
            error = goal - backDistance.get(); /**< calculate error normally */
        }

        derivative = error - prevError; /**< calculate derivative                               */
        prevError = error; /**< update previous error                                           */
        totalError += error; /**< update total error                                            */
        slew += kJ; /**< update acceleration                                                    */
        motorPower = error * kP + totalError * kI + derivative * kD; /**< calculate motor power */

        /** break out of the loop if necessary */
        if (fabs(error) < 40) {
            break;
        }

        /** slow down the acceleration of the robot if necessary */
        if (motorPower - prevMotorPower > slew) {
            motorPower = prevMotorPower + slew;
        }
        prevMotorPower = motorPower; /**< update the previous motor power */

        /** spin the motors */
        moveLeftDrivetrain(motorPower);
        moveRightDrivetrain(motorPower);
        
        /** delay to prevent all resources from being used */
        pros::delay(10);
    }
    /** stop all motors on the drivetrain */
    stopDrivetrain();
}


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
void backwardJPIDbackDistance(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime) 
{    
    double error = goal - backDistance.get(); /**< calculate the error               */
    double prevError = error; /**< the value of error at last cycle                  */
    double derivative = error - prevError; /**< the change in error since last cycle */
    double totalError = 0; /**< the total error since the start of the function      */
    double motorPower = 0; /**< the power to be sent to the drivetrain               */
    double prevMotorPower = 0; /**< the value of motorPower at last cycle            */
    double slew = kJ; /**< the maximum change in velocity                            */

    /** main loop */
    for (int time = 0; time < maxTime; time+=10) {

        /** calculate error, if statement to decide whether to use expectedDistance */
        if (time < clampOffset) {
            error = -expectedDistance; /**< calculate error using expectedDistance as position */
        } else {
            error = goal - backDistance.get(); /**< calculate error normally */
        }

        derivative = error - prevError; /**< calculate derivative                               */
        prevError = error; /**< update prevError                                                */
        totalError += error; /**< calculate error                                               */
        slew += kJ; /**< update acceleration                                                    */
        motorPower = error * kP + totalError * kI + derivative * kD; /**< calculate motor power */

        /**< break out of the loop if necessary */
        if (fabs(error) < 10) {
            break;
        }

        /** slew */
        if (motorPower - prevMotorPower < -slew) {
            motorPower = prevMotorPower - slew;
        }
        prevMotorPower = motorPower; /**< update the previous motor power */

        /** spin the motors */
        moveLeftDrivetrain(motorPower);
        moveRightDrivetrain(motorPower);
        
        /** delay to prevent all resources from being used */
        pros::delay(10);
    }
    /** stop all motors on the drivetrain */
    stopDrivetrain();
}


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
void backwardJPIDbackDistance2(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime) 
{    
    double error = goal - backDistance.get(); /**< calculate the error               */
    double prevError = error; /**< the value of error at last cycle                  */
    double derivative = error - prevError; /**< the change in error since last cycle */
    double totalError = 0; /**< the total error since the start of the function      */
    double motorPower = 0; /**< the power to be sent to the drivetrain               */
    double prevMotorPower = 0; /**< the value of motorPower at last cycle            */
    double slew = kJ; /**< the maximum change in velocity                            */

    /** main loop */
    for (int time = 0; time < maxTime; time+=10) {

        /** calculate error, if statement to decide whether to use expectedDistance */
        if (time < clampOffset) {
            error = -expectedDistance; /**< calculate error using expectedDistance as position */
        } else {
            error = goal - backDistance.get(); /**< calculate error normally */
        }

        derivative = error - prevError; /**< calculate derivative                               */
        prevError = error; /**< update prevError                                                */
        totalError += error; /**< calculate error                                               */
        slew += kJ; /**< update acceleration                                                    */
        motorPower = error * kP + totalError * kI + derivative * kD; /**< calculate motor power */

        /** slew */
        if (motorPower - prevMotorPower < -slew) {
            motorPower = prevMotorPower - slew;
        }
        prevMotorPower = motorPower; /**< update the previous motor power */

        /** spin the motors */
        moveLeftDrivetrain(motorPower);
        moveRightDrivetrain(motorPower);
        
        /** delay to prevent all resources from being used */
        pros::delay(10);
    }
    /** stop all motors on the drivetrain */
    stopDrivetrain();
}


/**
 * @brief function that calculates the error when turning, which is complex since it is bound by 0 to 360
 * 
 * @param goal the goal the robot wants to reach
 * @param position the current position of the robot
 * @return double 
 *
 */
double turnError(double goal, double position)
{
    /** if the target is closer to the right, turn right */
    if (std::fmod(position+180, 360) > goal && goal != position) {
        if (goal > position) {
            return goal - position;
        } else {
            return goal+360 - position;
        }

    /** if the target is closer to the left, turn left */
    } else if (std::fmod(position+180, 360) < goal && goal != position) {
        if (goal < position) {
            return goal - position;
        } else {
            return goal - (position+360);
        }
    
    /** else if goal equals position, return 0 */
    } else {
        return 0;
    }
}


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
void turnJPID(double goal, double kJ, double kP, double kI, double kD, double maxTime) 
{
    double error = turnError(goal, odom.get_heading(false)); /**< calculate the error */
    double prevError = error; /**< the value of error at last cycle                   */
    double derivative = error - prevError; /**< the change in error since last cycle  */
    double totalError = 0; /**< the total error since the start of the function       */
    double motorPower = 0; /**< the power to be sent to the drivetrain                */
    double prevMotorPower = 0; /**< the value of motorPower at last cycle             */
    double slew = 0; /**< the maximum change in velocity                              */

    /** main loop */
    for (int time = 0; time < maxTime; time+=10) {

        /** calculate variables */
        error = turnError(goal, odom.get_heading(false)); /**< calculate error                  */
        derivative = error - prevError; /**< calculate derivative                               */
        prevError = error; /**< update prevError                                                */
        totalError += error; /**< calculate error                                               */
        slew += kJ; /**< update acceleration                                                    */
        motorPower = error * kP + totalError * kI + derivative * kD; /**< calculate motor power */

        /** break out of the loop if necessary */
        if (fabs(error) < 1) {
            break;
        }

        /** spin the motors */
        moveLeftDrivetrain(motorPower);
        moveRightDrivetrain(-motorPower);
        
        /** delay to prevent all resources from being used */
        pros::delay(10);
    }
    /** stop all motors on the drivetrain */
    stopDrivetrain();
}


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
void turnJPID2(double goal, double kJ, double kP, double kI, double kD, double maxTime) 
{
    double error = goal - (imu1.get_rotation() + imu2.get_rotation())/2; // calculate the error
    double prevError = error; // the value of error at last cycle
    double derivative = error - prevError; // the change in error since last cycle
    double totalError = 0; // the total error since the start of the function
    double motorPower = 0; // the power to be sent to the drivetrain
    double prevMotorPower = 0; // the value of motorPower at last cycle
    double slew = 0; // the maximum change in velocity
    double recordedTime = 0;
    bool stopLoop = false;

    for (int time = 0; time < maxTime; time+=10) {

        /** calculate variables */
        error = goal - (imu1.get_rotation() + imu2.get_rotation())/2;; /**< calculate error     */
        derivative = error - prevError; /**< calculate derivative                               */
        prevError = error; /**< update prevError                                                */
        totalError += error; /**< calculate total error                                         */
        slew += kJ; /**< update acceleration                                                    */
        motorPower = error * kP + totalError * kI + derivative * kD; /**< calculate motor power */

        // break out of the loop if necessary
        if (fabs(error) < 0.5 && !stopLoop) {
            stopLoop = true;
            recordedTime = time;
        } else if (fabs(error) < 0.5 && stopLoop) {
            if (time - recordedTime >= 50) {
                break;
            }
        } else {
            stopLoop = false;
        }

        /** spin the motors */
        moveLeftDrivetrain(motorPower);
        moveRightDrivetrain(-motorPower);

        /** delay to prevent all resources from being used */
        pros::delay(10);
    }
    /** stop all motors on the drivetrain */
    stopDrivetrain();
}


/**
 * @brief function that aligns the robot with a tracked object
 * 
 * @param sigID signature ID number 
 * @param sig pointer to the signature
 * @param turnKP proportional
 * @param timeout timeout
 *
 */
void forwardVisionTracking(int sigID, pros::vision_signature_s_t* sig, double turnKP, double timeout) 
{
    /** set the signature ID and signature object */
    frontVision.set_signature(sigID, sig);

    /** set the zero point for the front vision sensor */
    frontVision.set_zero_point(pros::E_VISION_ZERO_CENTER);

    /** initialize variables for turn PID */
    double error = 0;
    double motorPower = 0;

    /** timestamp of when the function started */
    double startTime = pros::millis();

    /** main loop */
    while (true) {
        /** get the tracked object data */
        pros::vision_object_s_t trackedSig = frontVision.get_by_sig(0, sigID);
        
        /** if the distance sensor senses the mogo */
        error = 15 - trackedSig.x_middle_coord;

        /** break out of the loop if necessary */
        if (pros::millis() - startTime > timeout) {
            break;
        }
        
        /** calculate the power at which to spins the motors */
        motorPower = error * turnKP;

        /** spin the motors */
        moveLeftDrivetrain(-motorPower);
        moveRightDrivetrain(motorPower);

        /** delay to prevent all resources from being used */
        pros::delay(10);
    }
    /** stop all motors on the drivetrain */
    stopDrivetrain();
}


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
void back_vision_align(int sigID, pros::vision_signature_s_t* sig, double turnKP, double turnKI, double turnKD, double maxTime) 
{
    /** set the signature ID and signature object */
    backVision.set_signature(sigID, sig);

    /** set the zero point for the front vision sensor */
    backVision.set_zero_point(pros::E_VISION_ZERO_CENTER);

    /** initialize variables for turn PID */
    double error = 0; /**< distance between the target and the robot  */
    double prevError = 0; /**< the value of error at the last cycle   */
    double derivative = error - prevError; /**< derivative            */
    double totalError = 0; /**< total error                           */
    double motorPower = 0; /**< the power at which to spin the motors */

    /** main loop */
    for (int i = 0; i < maxTime; i+=10) {

        /** get the tracked object data */
        pros::vision_object_s_t trackedSig = backVision.get_by_sig(0, sigID);

        /** update variables */
        error = 0 - trackedSig.x_middle_coord;
        derivative = error - prevError;
        prevError = error;
        totalError += error;
        motorPower = error * turnKP + totalError * turnKI + derivative * turnKD;

        /** spin the motors */
        moveLeftDrivetrain(motorPower);
        moveRightDrivetrain(-motorPower);

        /** delay so it does not starve the cpu of resources */
        pros::delay(10);
    }
    /** stop the drivetrain if it has not stopped already */
    stopDrivetrain();
}


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

void basicForwardJPID(double goal, double kJ, double kP, double kI, double kD, double maxTime) 
{
    /** variables */
    double startPos = sideTrackingWheel.get_value() * ((M_PI*2.75)/360); /**< the position of the tracking wheel, in inches, at the start of the function (cannot tare the value of the tracking wheel cuz odom) */
    double error = goal - sideTrackingWheel.get_value()*((M_PI*2.75)/360) + startPos; /**< calculate error                                                                                                       */
    double prevError = error; /**< the value of error at the last cycle                                                                                                                                          */
    double derivative = error - prevError; /**< the change in error since the last cycle                                                                                                                         */
    double totalError = 0; /**< total error                                                                                                                                                                      */
    double slew = kJ; /**< maximum change in velocity                                                                                                                                                            */
    double motorPower = error * kP + totalError * kI + derivative * kD; /**< the power at which to spin the motors                                                                                               */
    double prevMotorPower = 0; /**< the power at which to spin the motors                                                                                                                                        */

    /** main loop */
    for (int i = 0; i < maxTime; i+=10) {
        
        /** update variables */
        error = goal - sideTrackingWheel.get_value()*((M_PI*2.75)/360) + startPos; /**< update error               */
        derivative = error - prevError; /**< the change in error since the last cycle                              */
        totalError += error; /**< update integral                                                                  */
        slew += kJ; /**< update acceleration                                                                       */
        motorPower = error*kP + totalError*kI + derivative * kD; /**< update the power at which to spin the motors */

        /** decide whether to limit acceleration */
        if (motorPower - prevMotorPower > slew) {
            motorPower = prevMotorPower + slew;
        }
        prevError = error; /**< update the previous error */

        /** decide whether to break out of the loop */
        if (fabs(error) < 0.1) {
            break;
        }

        /** spin the motors */
        moveLeftDrivetrain(motorPower); 
        moveRightDrivetrain(motorPower);

        /** delay so RTOS does not freeze */
        pros::delay(10);
    }
    /** stop the drivetrain if it has not stopped already */
    stopDrivetrain();
}


// function used to balance the robot
void balance(double speed, double target, double timeOffset)
{    
    double kP = 11;
    double kI = 0;
    double kD = 0;

    double error = target - (imu1.get_roll() + -imu2.get_roll())/2;
    double prevError = error;
    double derivative = 0;
    double totalError = error;
    double motorPower = 0;

    while (true) {
        error = target - (imu1.get_roll() + -imu2.get_roll())/2;
        derivative = error - prevError;
        prevError = error;
        totalError += error;
        motorPower = error*kP + totalError*kI + derivative*kD;
        
        if (error < 0) {
            moveLeftDrivetrain(-motorPower);
            moveRightDrivetrain(-motorPower);
        } else {
            stopDrivetrain();
        }

        pros::delay(10);
    }
}


// vision tracking functions
void forwardVisionTracking(int sigID, pros::vision_signature_s_t* sig, double turnGoal, double turnCutoffDistance, double turnKP, double turnKI, double turnKD, double forwardGoal, double forwardKP, double forwardKI, double forwardKD, double expectedDistance, double clampOffset, double timeout) 
{
    // set the signature ID and signature object
    frontVision.set_signature(sigID, sig);
    // set the zero point for the front vision sensor
    frontVision.set_zero_point(pros::E_VISION_ZERO_CENTER);
    // the object the vision sensor detected 
    pros::vision_object_s_t trackedSig = frontVision.get_by_sig(0, sigID);

    // initialize variables for forward PID
    double forwardError = forwardGoal - expectedDistance;
    double forwardPrevError = forwardError;
    double forwardDerivative = forwardError - forwardPrevError;
    double forwardTotalError = 0;
    double forwardMotorPower = forwardError*forwardKP + forwardTotalError*forwardKI + forwardDerivative*forwardKD;

    // initialize variables for turn PID
    double turnError = turnGoal - trackedSig.x_middle_coord;
    double turnPrevError = turnError;
    double turnDerivative = turnError - turnPrevError;
    double turnTotalError = 0;
    double turnMotorPower = turnError*turnKP + turnTotalError*turnKI + turnDerivative*turnKD;

    // main loop
    for (int time = 0; time < timeout; time+=10) {

        // update vision sensor tracked object
        trackedSig = frontVision.get_by_sig(0, sigID);

        // calculate forward PID variables
        if (time < clampOffset) {
            forwardError = expectedDistance;
        } else if (frontDistance.get() != 0) {
            forwardError = forwardGoal - -frontDistance.get();
        }
        forwardDerivative = forwardError - forwardPrevError;
        forwardPrevError = forwardError;
        forwardTotalError += forwardError; 
        forwardMotorPower = forwardError*forwardKP + forwardTotalError*forwardKI + forwardDerivative*forwardKD;

        // calculate turn PID variables
        if (false) {
            turnMotorPower = 0;
        } else {
            turnError = turnGoal - trackedSig.x_middle_coord;
            turnDerivative = turnError - turnPrevError;
            turnPrevError = turnError;
            turnTotalError += turnError;
            turnMotorPower = turnError*turnKP + turnTotalError*turnKI + turnDerivative*turnKD;
        }

        // move the drivetrain
        moveRightDrivetrain(forwardMotorPower + turnMotorPower);
        moveLeftDrivetrain(forwardMotorPower - turnMotorPower);

        pros::lcd::print(7, "forward error: %f", forwardError);
        
        // delay so RTOS does not freak out
        pros::delay(10);
    }

    stopDrivetrain();
}


// secondary function the align the robot using the vision sensor
void frontVisionAlign2(int sigID, pros::vision_signature_s_t* sig, double goal, double range, double breakTime, double kP, double maxSpeed, double timeout) {

    // set the signature ID and signature object
    frontVision.set_signature(1, &f_y_mogo_ald_up);
    // set the zero point for the front vision sensor
    frontVision.set_zero_point(pros::E_VISION_ZERO_CENTER);
    // the object the vision sensor detected 
    pros::vision_object_s_t trackedSig = frontVision.get_by_sig(0, sigID);

    // variables used for the PID
    double error = 0;
    double motorPower = error * kP;
    double prevMotorPower = 0;
    bool inRange = false;
    double rangeStart = 0;

    // main loop
	for (int time = 0; time < timeout; time+=10) {
        // update tracked object
		trackedSig = frontVision.get_by_sig(0, sigID);

        // calculate variables
        error = goal - trackedSig.x_middle_coord;
        motorPower = error * kP;

        if (motorPower > maxSpeed) {
            motorPower = maxSpeed;
        } else if (motorPower < -maxSpeed) {
            motorPower = -maxSpeed;
        }

        if (fabs(error) < range) {
            if (inRange == false) {
                inRange = true;
                rangeStart = time;
            } else if (time - rangeStart > breakTime) {
                break;
            }
        } else {
            inRange = false;
        }


        // spin the motors
        moveLeftDrivetrain(-motorPower);
        moveRightDrivetrain(motorPower);

		pros::lcd::print(7, "error: %f", error);

		pros::delay(10);
	}
    stopDrivetrain();
}