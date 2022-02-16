#include "main.h"
#include "pros/motors.h"
#include "pros/vision.h"
#include "robot_config.hpp"
#include "autoFunctions/basicOdomPID.hpp"
#include "autoFunctions/subsystems.hpp"
#include "autos.hpp"

using namespace pros; 


void skillsAuto()
{
    // reverse and clamp the alliance mogo
    backwardJPIDbackDistance(-118, 0, 0, 0.1, 0.2, 0.00006, 0, 1500);
    backClamp.set_value(true);
    frontClamp.set_value(true);
    // go forward slightly to allow for a better clamp while flipping out the conveyor
    pros::Task lambdaTask1{[=] { conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE); conveyor.move(-600); pros::delay(100); conveyor.move(0); pros::delay(100); }};
    basicForwardJPID(20, 3, 6, 0, 0, 1000);
    // unclamp the back mogo
    backClamp.set_value(false);
    // go forward a bit more
    basicForwardJPID(5, 3, 6, 0, 0, 1000);
    // go back to clamp the mogo
    backwardJPIDbackDistance(-118, 0, 0, 0.1, 0.2, 0.00006, 0, 1000);
    // clamp the mogo
    backClamp.set_value(true);
    // score the preloads
    conveyor.move(600);

    // turn to face the neutral mogo
    turnJPID2(92, 0, 1.5, 0, 0, 2000);
    // correct heading
    forwardVisionTracking(1, &front_yellow_mogo_alderfeild, 0.75, 2500);
    // grab the neutral mogo
    forwardJPIDfrontDistance(10, 0, 0, 1, 0.24, 0, 0, 2500); // 0.22
    // clamp the mogo
    frontClamp.set_value(false);

    // lift up the mogo
    moveLift(11700, 1, 0, 0, 3000);
    // turn left to face the ramp
    turnJPID2((imu1.get_rotation()+imu2.get_rotation())/2+37, 0, 1.5, 0, 0, 2000);
    // go towards the ramp while dropping the alliance mogo
    pros::Task lambdaTask2{[=] { pros::delay(700); backClamp.set_value(false); }};
    basicForwardJPID(60, 1, 6, 0, 0, 2000);
    // move down the lift
    moveLift(7000, 1, 0, 0, 3000);
    // unclamp the mogo
    frontClamp.set_value(true);

    // reverse a bit
    basicForwardJPID(-3, 1, 10, 0, 0, 500);
    // put up the lift 
    moveLift(11700, 1, 0, 0, 3000);
    // reverse 10 inches while putting down the lift
    pros::Task lambdaTask3{[=] { moveLift(0, 1, 0, 0, 4000); }};
    basicForwardJPID(-10, 1, 6, 0, 0, 1000);
    // turn to face the blue mogo
    turnJPID2((imu1.get_rotation() + imu2.get_rotation())/2 - 180, 0, 1, 0, 0, 2000);
    // delay for 500 ms
    pros::delay(500);
    // correct heading with the vision sensor 
    forwardVisionTracking(1, &front_blue_mogo_alderfeild, 0.75, 2500);
    // go and pick up the alliance mogo
    forwardJPIDfrontDistance(10, 0, 0, 1, 0.24, 0, 0, 2500); // 0.22
    // clamp the mogo
    frontClamp.set_value(false);

    // move up the lift
    moveLift(11700, 1, 0, 0, 3000);
    // turn towards the ramp
    turnJPID2((imu1.get_rotation() + imu2.get_rotation())/2 - 170, 0, 1, 0, 0, 2000);
    // go towards the ramp
    basicForwardJPID(60, 1, 6, 0, 0, 2000);
    // move down the lift
    moveLift(7000, 1, 0, 0, 3000);
    // unclamp the mogo
    frontClamp.set_value(true);
    // reverse a bit
    basicForwardJPID(-3, 1, 10, 0, 0, 500);
    // put up the lift 
    moveLift(11700, 1, 0, 0, 3000);
    // reverse 10 inches while putting down the lift
    pros::Task lambdaTask4{[=] { moveLift(0, 1, 0, 0, 4000); }};
    basicForwardJPID(-5, 1, 6, 0, 0, 1000);

}