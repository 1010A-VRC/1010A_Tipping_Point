#include "main.h"
#include "pros/vision.h"
#include "robot_config.hpp"
#include "autoFunctions/odometry.hpp"
#include "autoFunctions/basicOdomPID.hpp"

using namespace pros;


// function that moves the left side of the drivetrain
void moveLeftDrivetrain(double speed) {
    l1.move(speed);
    l2.move(speed);
    l3.move(speed);
}

// function that moves the right side of the drivetrain
void moveRightDrivetrain(double speed) {
    r1.move(speed);
    r2.move(speed);
    r3.move(speed);
}

// function that stops the drivetrain 
void stopDrivetrain() {
    l1.move(0);
    l2.move(0);
    l3.move(0);
    r1.move(0);
    r2.move(0);
    r3.move(0);
}


// forward odom PID
void forwardJPIDTrack(double goalX, double goalY, double kJ, double kP, double kI, double kD, double maxTime) 
{
    double newGoal = sideTrackingWheel.get_value() + (std::sqrt(std::pow(goalX - odom.get_x(), 2) + std::pow(goalY - odom.get_y(), 2)) / (M_PI*2.75))*360; // get the distance between the robot and the target point
    double error = newGoal - sideTrackingWheel.get_value(); // calculate the error
    double prevError = error; // the value of error at last cycle
    double derivative = error - prevError; // the change in error since last cycle
    double totalError = 0; // the total error since the start of the function
    double motorPower = 0; // the power to be sent to the drivetrain
    double prevMotorPower = 0; // the value of motorPower at last cycle
    double slew = kJ; // the maximum change in velocity

    for (int time = 0; time < maxTime; time+=10) {

        // calculate stuff
        error = newGoal - sideTrackingWheel.get_value(); // calculate error
        derivative = error - prevError; // calculate derivative
        prevError = error; // update prevError
        totalError += error; // calculate error
        slew += kJ;
        motorPower = error * kP + totalError * kI + derivative * kD; // calculate motor power

        // break out of the loop if necessary
        if (fabs(error) < 10) {
            break;
        }

        // slew
        if (motorPower - prevMotorPower > slew) {
            motorPower = prevMotorPower + slew;
        }
        prevMotorPower = motorPower;

        // spin the motors
        moveLeftDrivetrain(motorPower);
        moveRightDrivetrain(motorPower);
        std::printf("e: %f", newGoal);
        
        pros::delay(10);
    }
    stopDrivetrain();
}



// forward PID using front distance sensor
void forwardJPIDfrontDistance(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime) 
{
    double error = 0; // calculate the error
    double prevError = expectedDistance; // the value of error at last cycle
    double derivative = error - prevError; // the change in error since last cycle
    double totalError = 0; // the total error since the start of the function
    double motorPower = 0; // the power to be sent to the drivetrain
    double prevMotorPower = 0; // the value of motorPower at last cycle
    double slew = kJ; // the maximum change in velocity

    for (int time = 0; time < maxTime; time+=10) {

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

        derivative = error - prevError; // calculate derivative
        prevError = error; // update prevError
        totalError += error; // calculate error
        slew += kJ;
        motorPower = error * kP + totalError * kI + derivative * kD; // calculate motor power

        // break out of the loop if necessary
        if (fabs(error) < 40) {
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



// forward PID using back distance sensor
void forwardJPIDbackDistance(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime) 
{
    double error = goal - backDistance.get(); // calculate the error
    double prevError = error; // the value of error at last cycle
    double derivative = error - prevError; // the change in error since last cycle
    double totalError = 0; // the total error since the start of the function
    double motorPower = 0; // the power to be sent to the drivetrain
    double prevMotorPower = 0; // the value of motorPower at last cycle
    double slew = kJ; // the maximum change in velocity

    for (int time = 0; time < maxTime; time+=10) {

        // calculate stuff
        if (time < clampOffset) {
            error = expectedDistance;
        } else {
            error = goal - backDistance.get(); // calculate error
        }

        derivative = error - prevError; // calculate derivative
        prevError = error; // update prevError
        totalError += error; // calculate error
        slew += kJ;
        motorPower = error * kP + totalError * kI + derivative * kD; // calculate motor power

        // break out of the loop if necessary
        if (fabs(error) < 40) {
            break;
        }

        // slew
        if (motorPower - prevMotorPower > slew) {
            motorPower = prevMotorPower + slew;
        }
        prevMotorPower = motorPower;

        // spin the motors
        moveLeftDrivetrain(motorPower);
        moveRightDrivetrain(motorPower);
        
        pros::delay(10);
    }
    stopDrivetrain();
}



// backward odom PID
void backwardJPIDTrack(double goalX, double goalY, double kJ, double kP, double kI, double kD, double maxTime) 
{
    
    double newGoal = sideTrackingWheel.get_value() - (std::sqrt(std::pow(goalX - odom.get_x(), 2) + std::pow(goalY - odom.get_y(), 2)) / (M_PI*2.75))*360; // get the distance between the robot and the target point
    double error = newGoal - sideTrackingWheel.get_value(); // calculate the error
    double prevError = error; // the value of error at last cycle
    double derivative = error - prevError; // the change in error since last cycle
    double totalError = 0; // the total error since the start of the function
    double motorPower = 0; // the power to be sent to the drivetrain
    double prevMotorPower = 0; // the value of motorPower at last cycle
    double slew = kJ; // the maximum change in velocity

    for (int time = 0; time < maxTime; time+=10) {

        // calculate stuff
        error = newGoal - sideTrackingWheel.get_value(); // calculate error
        derivative = error - prevError; // calculate derivative
        prevError = error; // update prevError
        totalError += error; // calculate error
        slew += kJ;
        motorPower = error * kP + totalError * kI + derivative * kD; // calculate motor power

        // break out of the loop if necessary
        if (fabs(error) < 10) {
            break;
        }

        // slew
        if (motorPower - prevMotorPower < -slew) {
            motorPower = prevMotorPower - slew;
        }
        prevMotorPower = motorPower;

        moveLeftDrivetrain(motorPower);
        moveRightDrivetrain(motorPower);
        
        pros::delay(10);
    }
    stopDrivetrain();
}



// backward odom PID
void backwardJPIDbackDistance(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime) 
{    
    double error = goal - backDistance.get(); // calculate the error
    double prevError = error; // the value of error at last cycle
    double derivative = error - prevError; // the change in error since last cycle
    double totalError = 0; // the total error since the start of the function
    double motorPower = 0; // the power to be sent to the drivetrain
    double prevMotorPower = 0; // the value of motorPower at last cycle
    double slew = kJ; // the maximum change in velocity

    for (int time = 0; time < maxTime; time+=10) {

        // calculate stuff
        if (time < clampOffset) {
            error = expectedDistance;
        } else {
            error = goal - backDistance.get(); // calculate error
        }
        derivative = error - prevError; // calculate derivative
        prevError = error; // update prevError
        totalError += error; // calculate error
        slew += kJ;
        motorPower = error * kP + totalError * kI + derivative * kD; // calculate motor power

        // break out of the loop if necessary
        if (fabs(error) < 10) {
            break;
        }

        // slew
        if (motorPower - prevMotorPower < -slew) {
            motorPower = prevMotorPower - slew;
        }
        prevMotorPower = motorPower;

        moveLeftDrivetrain(motorPower);
        moveRightDrivetrain(motorPower);
        
        pros::delay(10);
    }
    stopDrivetrain();
}



// function that calculates the error when turning, which is complex since it is bound by 0 to 360
double turnError(double goal, double position)
{
    // if the target is closer to the right, turn right
    if (std::fmod(position+180, 360) > goal && goal != position) {
        if (goal > position) {
            return goal - position;
        } else {
            return goal+360 - position;
        }

    // if the target is closer to the left, turn left
    } else if (std::fmod(position+180, 360) < goal && goal != position) {
        if (goal < position) {
            return goal - position;
        } else {
            return goal - (position+360);
        }
    
    // else if goal equals position, return 0
    } else {
        return 0;
    }
}


// PID that turns the robot
void turnJPID(double goal, double kJ, double kP, double kI, double kD, double maxTime) 
{
    double error = turnError(goal, odom.get_heading(false)); // calculate the error
    double prevError = error; // the value of error at last cycle
    double derivative = error - prevError; // the change in error since last cycle
    double totalError = 0; // the total error since the start of the function
    double motorPower = 0; // the power to be sent to the drivetrain
    double prevMotorPower = 0; // the value of motorPower at last cycle
    double slew = 0; // the maximum change in velocity

    for (int time = 0; time < maxTime; time+=10) {

        // calculate stuff
        error = turnError(goal, odom.get_heading(false)); // calculate error
        derivative = error - prevError; // calculate derivative
        prevError = error; // update prevError
        totalError += error; // calculate error
        slew += kJ;
        motorPower = error * kP + totalError * kI + derivative * kD; // calculate motor power

        // break out of the loop if necessary
        if (fabs(error) < 1) {
            break;
        }

        // spin the motors
        moveLeftDrivetrain(motorPower);
        moveRightDrivetrain(-motorPower);
        printf("%f", motorPower);
        
        pros::delay(10);
    }
    stopDrivetrain();
}





// front vision sensor interpolator. This function interpolates the values of centerX for the front vision sensor since it is off-center from the mobile goal
int yInterpolator(double xValue) {

    // variables
    double closestLowX = 0;
    double closestHighX = 0;
    double closestLowY = 0;
    double closestHighY = 0;
    double finalOutput = 0;
    double slope = 0;
    std::vector<double> inputX = {300, 400, 500, 600, 700, 800, 900, 1000, 1100};
    std::vector<double> inputY = {206, 204, 200, 197, 195, 192, 190, 187, 185};

    // find the 2 closest points
    for (int i = 0; i < inputX.size(); i++) {
        // if the point is greater than closestLowX and less than xValue
        if (inputX.at(i) > closestLowX && inputX.at(i) < xValue) {
            closestLowX = inputX.at(i);
            closestLowY = inputY.at(i);
        // else if the value is greater than xValue, save it and break out of the loop
        } else if (inputX.at(i) > xValue) {
            closestHighX = inputX.at(i);
            closestHighY = inputY.at(i);
            break;
        // else if the xValue is equal to the current x value in the vector
        } else if (inputX.at(i) == xValue) {
            finalOutput = inputY.at(i);
            break;
        }
    }

    // check whether it should interpolate or not
    if (xValue > inputX.at(inputX.size()-1)) {
        finalOutput = inputY.at(inputY.size()-1);
    }

    // calculate the slope, if necessary
    if (finalOutput == 0) {
        slope = (closestHighY - closestLowY)/(closestHighX - closestLowX);
        finalOutput = slope * (xValue - closestHighX) + closestHighY;
    }
  
    // return the rounded finalOutput value
    return std::round(finalOutput) - 158;
}



// vision tracking functions
void forwardVisionTracking(int sigID, pros::vision_signature_s_t* sig, double turnKP) 
{
    // set the signature ID and signature object
    frontVision.set_signature(sigID, sig);

    // set the zero point for the front vision sensor
    frontVision.set_zero_point(pros::E_VISION_ZERO_CENTER);

    // initialize variables for turn PID
    double error = 0;
    double motorPower = 0;

    while (true) {
        pros::vision_object_s_t trackedSig = frontVision.get_by_sig(0, sigID);
        
        // if the distance sensor senses the mogo
        if (frontDistance.get() < 1500) {
            error = yInterpolator(frontDistance.get()) - trackedSig.x_middle_coord;
        } else {
            error = 10 - trackedSig.x_middle_coord;
        }
        
        printf("error: %f", error);
        motorPower = error * turnKP;

        moveLeftDrivetrain(-motorPower + 20);
        moveRightDrivetrain(motorPower + 20);


        pros::delay(10);
    }
}



// back vision aligning function 
void back_vision_align(int sigID, pros::vision_signature_s_t* sig, double turnKP, double turnKI, double turnKD, double maxTime) 
{
    // set the signature ID and the signature object 
    backVision.set_signature(sigID, sig);

    // set the zero point for the back vision sensor
    backVision.set_zero_point(pros::E_VISION_ZERO_CENTER);

    // initialize variables 
    double error = 0;
    double prevError = 0;
    double derivative = error - prevError;
    double totalError = 0;
    double motorPower = 0;

    // main loop
    for (int i = 0; i < maxTime; i+=10) {

        // update tracked object
        pros::vision_object_s_t trackedSig = backVision.get_by_sig(0, sigID);

        // update variables
        error = 0 - trackedSig.x_middle_coord;
        derivative = error - prevError;
        prevError = error;
        totalError += error;
        motorPower = error * turnKP + totalError * turnKI + derivative * turnKD;

        moveLeftDrivetrain(motorPower);
        moveRightDrivetrain(-motorPower);

        pros::delay(10);
    }
}