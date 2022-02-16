#include "main.h"
#include "pros/motors.h"
#include "pros/vision.h"
#include "robot_config.hpp"
#include "autoFunctions/basicOdomPID.hpp"
#include "autoFunctions/subsystems.hpp"
#include "autos.hpp"

using namespace pros;


void left2G() 
{
    // deactivate the front clamp
    pros::Task lambdaTask1{[=] { frontClamp.set_value(true); }};
    // go forward 
    forwardJPIDfrontDistance(10, 1400, 300, 100, 0.3, 0, 0, 1100); // 0.22
    // activate the clamp
    frontClamp.set_value(false);

    // go back while simultaneously flipping out the ring guide
    pros::Task lambdaTask2{[=] { conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE); conveyor.move(-600); pros::delay(100); conveyor.move(0); pros::delay(100); }};
    basicForwardJPID(-40, 3, 6, 0, 0, 1500);
    // turn right to face the alliance mogo
    turnJPID2(105, 0, 2, 0, 0, 1500);
    // reverse and clamp the alliance mogo
    backwardJPIDbackDistance(-118, 0, 0, 0.1, 0.2, 0.00006, 0, 1500);
    backClamp.set_value(true);

    // go forward slightly to allow for a better clamp
    basicForwardJPID(20, 3, 6, 0, 0, 1000);
    // unclamp the back mogo
    backClamp.set_value(false);
    // go forward a bit more
    basicForwardJPID(10, 3, 6, 0, 0, 1000);
    // go back to clamp the mogo
    backwardJPIDbackDistance(-118, 0, 0, 0.1, 0.2, 0.00006, 0, 1000);
    // clamp the mogo
    backClamp.set_value(true);
    // score the preloads
    conveyor.move(600);
}