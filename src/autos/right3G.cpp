#include "main.h"
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
    // flip out the ring guide
    pros::Task lambdaTask2{[=] { conveyor.tare_position(); conveyor.move_absolute(90, 600); }};
    // go forward 
    forwardJPIDfrontDistance(10, 1400, 300, 1, 0.2, 0, 0, 5000);
    // activate the clamp
    frontClamp.set_value(false);

    // lift the front mogo up to stop it from interacting with the rings
    pros::Task lambda3{[=] { moveLift(3000, 1, 0.000001, 0, 10000); }};
    // go back
    backwardJPIDbackDistance(1175, 1500, 0, 0.5, 0.22, 0, 0, 5000);

    turnJPID(108, 0.1, 0.9, 0, 0, 5000);

    backwardJPIDbackDistance(36, 0, 0, 0.5, 0.22, 0, 0, 0);
}
