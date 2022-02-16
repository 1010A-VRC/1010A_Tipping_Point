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
    // go forward 
    forwardJPIDfrontDistance(10, 1400, 300, 1, 0.26, 0, 0, 5000); // 0.22
    // activate the clamp
    frontClamp.set_value(false);

    // lift the front mogo up to stop it from interacting with the rings
    pros::Task lambdaTask2{[=] { moveLift(3000, 1, 0.000001, 0, 10000); }};
    // go back
    backwardJPIDbackDistance(720, 1500, 0, 0.5, 0.22, 0, 0, 15000);
    // align with middle mogo
    turnJPID(127, 0.1, 0.91, 0, 0, 1500);
    // go back and clamp the middle mogo
    backwardJPIDbackDistance(-125, 0, 0, 0.1, 0.12, 0.00006, 0, 2500);
    backClamp.set_value(true);

    // go forward a little bit
    basicForwardJPID(360, 0.1, 1, 0, 0, 1500);
    

}
