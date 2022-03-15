#include "main.h"
#include "pros/misc.hpp"
#include "robot_config.hpp"
#include "subsystems.hpp"

// task for stopping the lift
pros::Task* liftStopper = nullptr;

// variable that decides whether the lift should stop or not
static bool liftStop = false;

void stopLift() {

    double goal = lift.get_position();
    double error = 0;
    double prevError = 0;
    double derivative = 0;
    double totalError = 0;
    double kP = 1;
    double kI = 0;
    double kD = 0;
    double motorPower = error * kP + totalError * kI + derivative * kD;

    // main loop
    while (pros::competition::is_autonomous() && liftStop) {
        
        // update variables
        error = goal - lift.get_position();
        derivative = error - prevError;
        prevError = error;
        totalError += error;
        motorPower = error * kP + totalError * kI + derivative * kD;

        // spin the motor
        lift.move(motorPower);

        // delay 
        pros::delay(10);
    }

    // when it is no longer autonomous, or the lift shouldn't be stopped, stop the task from running
    liftStopper->remove();
    master.rumble("-");
}


void moveLift(double goal, double kP, double kI, double kD, double maxTime) {

    // stop the liftStopper thread from running 
    liftStop = false;
    pros::delay(15);

    // delete the lift stopper pointer and set it to a nullptr
    if (liftStopper != nullptr) {
        delete liftStopper;
        liftStopper = nullptr;
    }

    // define variables 
    double error = goal - frontLiftRotation.get_angle();
    double prevError = error;
    double derivative = error - prevError; 
    double totalError = 0;
    double motorPower = error * kP + totalError * kI + derivative * kD;

    // main loop
    for (int i = 0; i < maxTime; i+=10) {

        // check if the lift is within the range of error
        if (fabs(error) < 100) {
            break;
        }

        // update variables
        error = goal - frontLiftRotation.get_angle();
        derivative = error - prevError;
        prevError = error;
        totalError += error;
        motorPower = error * kP + totalError * kI + derivative * kD;

        // move the lift
        lift.move(motorPower);

        // delay 
        pros::delay(10);
    }

    // upon exit, run the liftStop task
    liftStop = true;
    liftStopper = new pros::Task(stopLift);
}