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
#include "pros/misc.h"
#include "robot_config.hpp"
#include "opcontrol.hpp"

using namespace pros;


/**
 * @brief Method controlling the drivetrain
 * 
 */

bool normalDrivetrain = true;

void user_control::drivetrain_control() {
    
    /** variables */
    double joystickLeftY = 0; /**< left joystick Y position    */
    double joystickRightX = 0; /**< right joystick X position  */

    double forwardPower = 0; /**< forward motor power          */
    double turnPower = 0; /**< turning motor power             */

    double rightPower = 0; /**< right motor power              */
    double leftPower = 0; /**< left motor power                */

    double prevRightPower = 0; /**< previous right motor power */
    double prevLeftPower = 0; /**< previous left motor power   */
    double prevForwardPower = 0;
    double prevTurnPower = 0;

    double maxAccel = 20; /**< the maximum deceleration        */
    double maxDecel = 3; /**< the maximum acceleration        */

    double turnAccel = 50; /**< acceleration multiplier when turning */

    double tempTurnAccel = 0;

    bool aToggle = false;

    FILE* file = fopen("/usd/control.txt", "r");

    /** main loop */
    while (true) {
        while (normalDrivetrain) {

        /** decide whether to run single controller or dual controller */
  	    if (partnerController.is_connected()) {
  	        partner = &partnerController;
  	    } else {
  	        partner = &master;
  	    }

        /** get input from joystick */
        joystickLeftY = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y); /**< Left Y joystick    */
        joystickRightX = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X); /**< Right X joystick */

        /**< set power to zero if deadzone is reached          */
        if (fabs(joystickLeftY) < 1) {
            joystickLeftY = 0; /**< left Y joystick deadzone   */
        }
        if (fabs(joystickRightX) < 1) {
            joystickRightX = 0; /**< right X joystick deadzone */
        }

        /** calculate the motor voltage based on a cubic equation */
        forwardPower = pow(joystickLeftY, 3)/16129; /**< forward motor power */
        turnPower = pow(joystickRightX, 3)/16129; /**< turning motor power   */

        /** forward motor power rate limiter. Accelerates and decelerates the lateral power of the drivetrain */
        if (prevForwardPower < 0) { /**< if the robot is going backwards */
            if (forwardPower > prevForwardPower) { /**< if the user wants to decelerate backwards */
                if (forwardPower - prevForwardPower > maxDecel) { /**< if the user wants to decelerate backwards too fast */
                    forwardPower = prevForwardPower + maxDecel; /**< limit backwards deceleration */
                }
            } else if (forwardPower < prevForwardPower) { /**< if the user wants to accelerate backwards */
                if (forwardPower - prevForwardPower < -maxAccel) { /**< if the user wants to accelerate backwards too fast */
                    forwardPower = prevForwardPower - maxAccel; /**< limit backwards acceleration */
                }
            }
        } else { /**< else if the robot is going forwards or not moving */
            if (forwardPower > prevForwardPower) { /**< if the user wants to accelerate forwards */
                if (forwardPower - prevForwardPower > maxAccel) { /**< if the user wants to accelerate forwards too fast */
                    forwardPower = prevForwardPower + maxAccel; /**< limit forwards acceleration */
                }
            } else if (forwardPower < prevForwardPower) { /**< if the user wants to decelerate forwards */
                if (forwardPower - prevForwardPower < -maxDecel) { /**< if the user wants to decelerate forwards too fast */
                    forwardPower = prevForwardPower - maxDecel; /**< limit forwards acceleration */
                }
            }
        }

        /** calculate motor speeds */
        leftPower = forwardPower + turnPower; /** calculate left motor power   */
        rightPower = forwardPower - turnPower; /** calculate right motor power */

        /** spin the motors                     */
        l1.move(leftPower); /**< left 1 motor   */
        l2.move(leftPower); /**< left 2 motor   */
        l3.move(leftPower); /**< right 1 motor */
        r1.move(rightPower);
        r2.move(rightPower); /**< right 2 motor */
        r3.move(rightPower); /**< right 3 motor */

        pros::lcd::print(0, "speed: %f", l1.get_actual_velocity());

        //fputs("leftSpeed: %f, rightSpeed: %f, time: %d \r\n", file);

        /** update previous stored values */
        prevLeftPower = leftPower; /**< left side    */
        prevRightPower = rightPower; /**< right side */
        prevForwardPower = forwardPower;
        prevTurnPower = turnPower;

        /** delay so RTOS does not freeze */
        pros::delay(10);
        }

        while (!normalDrivetrain) {
            double forwardPower = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
            double turnPower = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

            double leftMotorPower = forwardPower + turnPower;
            double rightMotorPower = forwardPower - turnPower;

            l1.move(leftMotorPower);
            l2.move(leftMotorPower);
            l3.move(leftMotorPower);
            r1.move(rightMotorPower);
            r2.move(rightMotorPower);
            r3.move(rightMotorPower); 
        
            pros::delay(10);
        }

        pros::delay(10);
    }
}


void user_control::driveToggle() {
    while (true) {
        if (partnerController.get_digital(E_CONTROLLER_DIGITAL_R2)) {
            normalDrivetrain = !normalDrivetrain;
            while (partnerController.get_digital(E_CONTROLLER_DIGITAL_R2)) {
                pros::delay(10);
            }
        }
        pros::delay(10);
    }
}



/**
 * @brief Method using PID to keep the lift in positions
 * 
 */
void user_control::front_lift_stop()
{
    /** constants for the PID */
    double kP = 0.02;
    double kI = 0.0;
    double kD = 0.0;

    /** variables for the PID */
    int goal = frontLiftRotation.get_angle() + 500; // goal
    if (goal > E_FRONT_LIFT_MAX) {
        goal = E_FRONT_LIFT_MAX;
    }
    double motorSpeed;      /**< speed at which to move the lift motor                                          */
    double error;           /**< distance from the lift's current position and the target position              */
    double prevError = 0;   /**< previous distance from the lift's current position and the target position     */
    double totalError;      /**< total error from each loop */
    double derivative;      /**< change in error from previous loop, used to detect a large change in position  */

    /* loop while the exit condition is met for the front_lift_control main if statements */
    while((!master.get_digital(E_CONTROLLER_DIGITAL_L1)&&!master.get_digital(E_CONTROLLER_DIGITAL_L2)) || (master.get_digital(E_CONTROLLER_DIGITAL_L1)&&frontLiftRotation.get_angle()>=E_FRONT_LIFT_MAX-500) || (master.get_digital(E_CONTROLLER_DIGITAL_L2)&&frontLiftRotation.get_angle()<=E_FRONT_LIFT_HOME)) {

        /** update variables */
        error = goal - frontLiftRotation.get_position();        /**< error              */
        totalError += error;                                    /**< total error        */
        derivative = error - prevError;                         /**< derivative         */
        prevError = error;                                      /**< previous error     */
        motorSpeed = error*kP + totalError*kI + derivative*kD;  /**< motor speed        */

        /** move the lift */
        lift.move(motorSpeed);

        /** short delay so RTOS does not freeze */
        delay(10);
    }
}


/**
 * @brief Method controlling the front lift
 * 
 */
void user_control::front_lift_control()
{
    /** loop forever */
    while (true) {

        /** if controller button L1 is pressed, move the front lift up */
        if (master.get_digital(E_CONTROLLER_DIGITAL_L1) && frontLiftRotation.get_angle() <= E_FRONT_LIFT_MAX) {
            lift.move(std::numeric_limits<std::int32_t>::max()); /**< move the lift upwards at maximum power */

        /** else if controller button L2 is pressed, move the front lift down */
        } else if (master.get_digital(E_CONTROLLER_DIGITAL_L2) && frontLiftRotation.get_angle() >= E_FRONT_LIFT_HOME) {
            lift.move(std::numeric_limits<std::int32_t>::min()); /**< move the front lift down at maximum power */

        /** else hold the front lift at a positions */
        } else {
            front_lift_stop();
        }

        /** short delay so RTOS does not freeze */
        delay(10);
    }
}


/**
 * @brief Method controlling the front clamp
 * 
 */
void user_control::front_clamp_control()
{
    /** loop forever */
    while (true) {

        /** if controller button R1 is pressed, release the front clamp */
        if (master.get_digital(E_CONTROLLER_DIGITAL_R2)) {
            frontClamp.set_value(false);

        /** else if controller button R2 is pressed, actuate the front clamp */
        } else if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
            frontClamp.set_value(true);
        }

        /** short delay so RTOS does not freeze */
        delay(10);
    }
}


/**
 * @brief Method controlling the back clamp
 * 
 */
void user_control::back_clamp_control() 
{
    /** loop forever */
    while (true) {

        /** check if the partner controller is connected */
        if (partnerController.is_connected()) {
            /** if controller button up is pressed, release the back clamp */
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                backClamp.set_value(false);
            /** else if controller button down is pressed, actuate the back clamp */
            } else if (master.get_digital(E_CONTROLLER_DIGITAL_Y)) {
                backClamp.set_value(true);
            }

        /** if the partner controller is not connected */
        } else {
            /** if controller button up is pressed, release the back clamp */
            if (partner->get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                backClamp.set_value(false);
            /** else if controller button down is pressed, actuate the back clamp */
            } else if (partner->get_digital(E_CONTROLLER_DIGITAL_Y)) {
                backClamp.set_value(true);
            }
        }

        /** short delay so RTOS does not freeze */
        delay(10);
    }
}


/**
 * @brief Method controlling the conveyor
 * 
 */
void user_control::conveyor_control() 
{
    /** loop forever */
    while (true) {

        /** if controller button B was pressed, toggle the conveyor */
        if (partner->get_digital(E_CONTROLLER_DIGITAL_B)) {

            /** if the conveyor is not spinning forwards, spin it forwards */
            if (conveyor.get_voltage() <= 0) {
                /** move the conveyor forwards at maximum power */
                conveyor.move(std::numeric_limits<std::int32_t>::max());

            /** else if the conveyor is spinning forwards, stop it */
            } else if (conveyor.get_voltage() > 0) {
                /** stop the conveyor */
                conveyor.move(0);
            }

            /** while button b is pressed, halt the function so the conveyor does not spasm */
            while (partner->get_digital(E_CONTROLLER_DIGITAL_B)) {
                delay(10);
            }


        /** else if button X was pressed, toggle the conveyor */
        } else if (partner->get_digital(E_CONTROLLER_DIGITAL_A)) {

            /** if the conveyor is not spinning backwards, spin it backwards */
            if (conveyor.get_voltage() >= 0) {
                /** move the conveyor backwards at maximum power */
                conveyor.move(std::numeric_limits<std::int32_t>::min());

            /** else if the conveyor is spinning backwards, stop it */
            } else if (conveyor.get_voltage() < 0) {
                /** stop the conveyor  */
                conveyor.move(0);
            }

            /** while button X is pressed, halt the function so the conveyor does not spasm */
            while (partner->get_digital(E_CONTROLLER_DIGITAL_A)) {
                delay(10);
            }
        }

        /** small delay so RTOS does not freeze */
        delay(10);
    }
}