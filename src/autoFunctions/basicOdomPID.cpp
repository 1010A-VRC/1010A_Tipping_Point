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
        if (fabs(error) < 40) {
            totalPower /= time/10;
            pros::lcd::print(7, "average speed: %f", totalPower);
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
    double error = goal - backDistance.get(); // calculate the error // 30 - 1100
    double prevError = error; // the value of error at last cycle
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


// backward odom PID
void backwardJPIDbackDistance2(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime) 
{    
    double error = goal - backDistance.get(); // calculate the error // 30 - 1100
    double prevError = error; // the value of error at last cycle
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
            error = goal - backDistance.get(); // calculate error
        }
        derivative = error - prevError; // calculate derivative
        prevError = error; // update prevError
        totalError += error; // calculate error
        slew += kJ;
        motorPower = error * kP + totalError * kI + derivative * kD; // calculate motor power

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
        
        pros::delay(10);
    }
    stopDrivetrain();
}



// PID that turns the robot
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

        // calculate stuff
        error = goal - (imu1.get_rotation() + imu2.get_rotation())/2;; // calculate error
        derivative = error - prevError; // calculate derivative
        prevError = error; // update prevError
        totalError += error; // calculate error
        slew += kJ;
        motorPower = error * kP + totalError * kI + derivative * kD; // calculate motor power

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

        // spin the motors
        moveLeftDrivetrain(motorPower);
        moveRightDrivetrain(-motorPower);
        
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
void forwardVisionTracking(int sigID, pros::vision_signature_s_t* sig, double turnKP, double timeout) 
{
    // set the signature ID and signature object
    frontVision.set_signature(sigID, sig);

    while (true) {
        // the object the vision sensor detected 
        pros::vision_object_s_t trackedSig = frontVision.get_by_sig(0, sigID);
        pros::lcd::print(7, "center x: %d", trackedSig.x_middle_coord);
        pros::delay(10);
    }
    
    stopDrivetrain();
}



// vision tracking functions
void backVisionTracking(int sigID, pros::vision_signature_s_t* sig, double turnKP, double timeout) 
{
    // set the signature ID and signature object
    backVision.set_signature(sigID, sig);

    // set the zero point for the front vision sensor
    backVision.set_zero_point(pros::E_VISION_ZERO_CENTER);

    // initialize variables for turn PID
    double error = 0;
    double motorPower = 0;

    double startTime = pros::millis();

    while (true) {
        pros::vision_object_s_t trackedSig = backVision.get_by_sig(0, sigID);
        
        // if the distance sensor senses the mogo
        error = 15 - trackedSig.x_middle_coord;

        if (pros::millis() - startTime > timeout) {
            break;
        }
        
        motorPower = error * turnKP;

        moveLeftDrivetrain(-motorPower);
        moveRightDrivetrain(motorPower);

        pros::delay(10);
    }
    stopDrivetrain();
    pros::lcd::print(7, "error: %f", error);
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



// basic forward JIPD functions
void basicForwardJPID(double goal, double kJ, double kP, double kI, double kD, double maxTime) 
{
    // variables 
    double startPos = sideTrackingWheel.get_value() * ((M_PI*2.75)/360);
    double error = goal - sideTrackingWheel.get_value()*((M_PI*2.75)/360) + startPos;
    double prevError = error;
    double derivative = error - prevError;
    double totalError = 0;
    double slew = kJ;
    double motorPower = error * kP + totalError * kI + derivative * kD;
    double prevMotorPower = 0;

    for (int i = 0; i < maxTime; i+=10) {

        error = goal - sideTrackingWheel.get_value()*((M_PI*2.75)/360) + startPos;
        derivative = error - prevError;
        totalError += error;
        slew += kJ;
        motorPower = error*kP + totalError*kI + derivative * kD;

        if (motorPower - prevMotorPower > slew) {
            motorPower = prevMotorPower + slew;
        }
        prevError = error;


        if (fabs(error) < 0.1) {
            break;
        }

        moveLeftDrivetrain(motorPower); 
        moveRightDrivetrain(motorPower);

        pros::delay(10);
    }
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