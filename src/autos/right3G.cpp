#include "main.h"
#include "pros/motors.h"
#include "pros/vision.h"
#include "robot_config.hpp"
#include "autoFunctions/basicOdomPID.hpp"
#include "autoFunctions/subsystems.hpp"
#include "autos.hpp"

using namespace pros;

void right3G(pros::vision_signature_s_t* yellowMogo, double yellowMogoID, pros::vision_signature_s_t* allianceMogo, double allianceMogoID)
{   
    // deactivate the front clamp
    pros::Task lambdaTask1{[=] { frontClamp.set_value(true); }};
    // go forward 
    forwardJPIDfrontDistance(10, 1400, 300, 100, 0.4, 0, 0, 870); // 0.22
    // activate the clamp
    frontClamp.set_value(false);

    // lift the front mogo up to stop it from interacting with the rings
    pros::Task lambdaTask2{[=] { moveLift(3000, 1, 0.000001, 0, 10000); }};
    // go back
    backwardJPIDbackDistance(720, 1500, 0, 0.1, 0.22, 0, 0, 15000);
    // align with middle mogo
    turnJPID(127, 0.1, 0.91, 0, 0, 1500);
    // go back and clamp the middle mogo
    backwardJPIDbackDistance(-125, 0, 0, 0.1, 0.12, 0.00006, 0, 2500);
    backClamp.set_value(true);

    // go forward a little bit
    basicForwardJPID(20, 3, 6, 0, 0, 1500);
    // correct heading
    turnJPID2(140, 1000, 8, 0, 0, 1500);
    // go all the way forward while lifting the arm up and dropping off the tall neutral mobile goal
    pros::Task lambdaTask3{[=] { pros::delay(800); backClamp.set_value(false); }};
    pros::Task lambdaTask4{[=] { moveLift(11700, 1, 0.000001, 0, 10000); }};
    basicForwardJPID(50, 3, 6, 0, 0, 1500);

    // turn to face the alliance mogo while flipping out the ring guide
    pros::Task lambdaTask5{[=] { conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); conveyor.move(-600); pros::delay(100); conveyor.move(0); pros::delay(100); }};
    turnJPID2(223, 0.1, 1.2, 0, 0, 1500);
    // clamp the alliance mogo
    backwardJPIDbackDistance(-125, 0, 0, 0.1, 0.2, 0.00006, 0, 1500);
    // score the preloads
    conveyor.move(600);
    

}
