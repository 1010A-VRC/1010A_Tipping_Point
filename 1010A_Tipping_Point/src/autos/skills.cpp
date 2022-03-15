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
    backwardJPIDbackDistance(-118, 0, 0, 0.1, 0.2, 0.00006, 0, 800);
    backClamp.set_value(true);
    frontClamp.set_value(true);
    // go forward slightly to allow for a better clamp while flipping out the conveyor
    pros::Task lambdaTask1{[=] { conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE); conveyor.move(-600); pros::delay(100); conveyor.move(0); pros::delay(100); }};
    basicForwardJPID(15, 3, 6, 0, 0, 1000);
    // score the preloads
    conveyor.move(600);

    // turn to face the neutral mogo 
    turnJPID2(102, 0, 1.35, 0, 0, 750);
    turnJPID2(102, 0, 4, 0, 0.1, 750); // 1.35 kd = 0.1
    // go towards the neutral mobile goal 
    forwardJPIDfrontDistance(4, 0, 0, 1, 0.22, 0, 0, 2000); // 0.22
    // clamp the neutral mobile goal 
    frontClamp.set_value(false); 
    delay(50);

    // lift up the lift while turning to the right
    turnJPID2(90, 0, 4, 0, 0, 2000);
    
    pros::Task lambdaTask8{[=] { moveLift(10000, 1, 0, 0, 3000); }};
    // spin the conveyor 
    conveyor.move(127);
    // go towards some of the rings
    basicForwardJPID(35, 3, 6, 0, 0, 2000);
    // turn right
    turnJPID2(180, 0, 1.6, 0, 0, 1500);
    turnJPID2(180, 0, 8, 0, 0, 750);
    // go forwards
    basicForwardJPID(32, 3, 6, 0, 0, 2000);

    // turn left
    turnJPID2(90, 0, 2, 0, 0, 1500);
    turnJPID2(90, 0, 12, 0, 0, 750);
    // go forward a little bit
    basicForwardJPID(14.25, 3, 6, 0, 0, 1000);
    // move the lift down to balance the ramp (first mogo on the ramp)
    moveLift(6500, 1, 0, 0, 3000);

    delay(500);
    // unclamp 
    frontClamp.set_value(true);
    // go back a little bit 
    basicForwardJPID(-3, 3, 7.5, 0, 0, 1000);

    // move the lift up 
    moveLift(10000, 1, 0, 0, 1000);

    //finish going backwards
    basicForwardJPID(-4, 3, 7.5, 0, 0, 1000);
    pros::delay(250);

    // go back some more while putting the lift down
    pros::Task lambdaTask3{[=] { moveLift(0, 1, 0, 0, 3000); }};
    basicForwardJPID(-11, 3, 6, 0, 0, 1000); //used to be -13 but changed to prevent the robot from moving

    turnJPID2(100, 0, 1.3, 0, 0, 1500);
    turnJPID2(100, 0, 4, 0, 0, 750);
    // unclamp the back 
    backClamp.set_value(false);
    // go forward a little bit
    basicForwardJPID(10, 3, 6, 0, 0, 1000); //used to be 10 but supposed to be 8 but made it 10 to appaoch more

    // turn to face the back mogo
    turnJPID2(280, 0, 1.3, 0, 0, 1500);
    turnJPID2(280, 0, 4, 0, 0, 1000);
    // go forward to clamp the mogo
    forwardJPIDfrontDistance(4, 0, 0, 1, 0.4, 0, 0, 1500);
    frontClamp.set_value(false);
    basicForwardJPID(-5, 3, 6, 0, 0, 1000);

    // turn to face the ramp while lifting up the mogo
    pros::Task lambdaTask4{[=] { moveLift(11700, 1, 0, 0, 3000); }};
    turnJPID2(100, 0, 1.3, 0, 0, 1500);
    turnJPID2(100, 0, 4, 0, 0, 1000);

    // go forward a little bit
    basicForwardJPID(20, 3, 6, 0, 0, 1000);
    // move the lift down to balance the ramp
    moveLift(6000, 1, 0, 0, 3000);
    // unclamp 
    frontClamp.set_value(true);

    delay(200);

    // move the lift up 
    moveLift(11700, 1, 0, 0, 1000);

    delay(50);

    // go back a little bit 
    basicForwardJPID(-7, 3, 7.5, 0, 0, 1000);
    /*
    // move the lift up 
    moveLift(11700, 1, 0, 0, 3000);
    */
    pros::delay(250);
    // go back some more while putting the lift down
    pros::Task lambdaTask5{[=] { moveLift(0, 1, 0, 0, 3000); }};

    
    basicForwardJPID(-13, 3, 6, 0, 0, 1000); //used to be -15 but changed to -13 to prevent the robot from touching the tall mogo with back

    // turn to face the tall mogo
    turnJPID2(262, 0, 1.3, 0, 0, 1200);
    //turnJPID2(262, 0, 4, 0, 0, 750);
    //frontVisionAlign2(1, &f_y_mogo_board, 0, 2.5, 250, 4, 20, 2000);
    delay(500);
    forwardJPIDfrontDistance(4, 0, 0, 1, 0.5, 0, 0, 1500);

    //decrease sussiness with the delay 
    delay(100);
    frontClamp.set_value(false);

    delay(500);
    
    // turn to face the ramp while lifting up the mogo just a little so the mogo doesn't touch the ground
    moveLift(2000, 1, 0, 0, 3000);
    turnJPID2(67, 0, 1.3, 0, 0, 1200);
    turnJPID2(67, 0, 4, 0, 0, 750);
    // prepare the arm to put the mogo down 
    moveLift(10000, 1, 0, 0, 3000);
    //go forward to put the mogo down 
    basicForwardJPID(20, 3, 6, 0, 0, 1000);
    basicForwardJPID(20, 3, 6, 0, 0, 1000);
    delay(50);
    basicForwardJPID(15, 3, 6, 0, 0, 1000);
    // move the lift down so it is slightly above the ramp
    moveLift(7500, 1, 0, 0, 3000);

    // turn to the right to make space for the tall neutral mogo
    turnJPID2(160, 0, 1.3, 0, 0, 1000);
    turnJPID2(160, 0, 4, 0, 0, 1000);
    // turn to the left 
    turnJPID2(140, 0, 1.3, 0, 0, 1000);
    turnJPID2(140, 0, 4, 0, 0, 1000);
    // give the robot a chance to lose all its momentum
    delay(500);

    pros::Task lambdaTask9{[=] { basicForwardJPID(3, 3, 6, 0, 0, 1000);}};

    moveLift(6300, 1, 0, 0, 3000);
    // release the tall neutral mogo
    delay(1000);
    frontClamp.set_value(true);

    basicForwardJPID(-5, 3, 6, 0, 0, 1000);
    moveLift(10000, 1, 0, 0, 1000);
    basicForwardJPID(-10, 3, 6, 0, 0, 1000);
    moveLift(0, 1, 0, 0, 3000);
    
    //---------------------------------------------------------------------------------//
    //-----------------------------------1st section-----------------------------------//
    //---------------------------------------------------------------------------------//

    
    //face the blue mogo on the alliance diagonal line
    turnJPID2(175, 0, 1.3, 0, 0, 1000);
    turnJPID2(175, 0, 4, 0, 0, 750);

    //clamp blue mogo
    backwardJPIDbackDistance(-118, 0, 0, 0.1, 0.2, 0.00006, 0, 1300);
    backClamp.set_value(true);

    delay(100);

    turnJPID2(180, 0, 1.3, 0, 0, 1000);
    turnJPID2(180, 0, 4, 0, 0, 750);

    basicForwardJPID(92, 3, 4.5, 0, 0, 2000);

    turnJPID2(270, 0, 1.3, 0, 0, 1000);
    turnJPID2(270, 0, 4, 0, 0, 750);

    //---------------------------------------------------------------------------//
    //-------------directly aligning with the far neutral mogo-------------------//
    //---------------------------------------------------------------------------//
    /*
    //go a bit forward to align with the mogo
    basicForwardJPID(5, 3, 6, 0, 0, 1000);

    //Last Neutral Mogo robot heading
    turnJPID2(200, 0, 1.3, 0, 0, 1500);
    turnJPID2(200, 0, 4, 0, 0, 1500);

    
    //go towards the last neutral mogo and stopping where there are a lot of ring accumulations 
    basicForwardJPID(50, 3, 6, 0, 0, 2000);

    
    // turning +/- 20 degree to get rid of rings in the middle of the path to the last neutral mogo
    turnJPID2(230, 0, 1.3, 0, 0, 1500);
    turnJPID2(230, 0, 4, 0, 0, 1500);

    turnJPID2(250, 0, 1.3, 0, 0, 1500);
    turnJPID2(250, 0, 4, 0, 0, 1500);

    //make the robot head towards the last neutral mogo after clearing the rings
    turnJPID2(210, 0, 1.3, 0, 0, 1500);
    turnJPID2(210, 0, 4, 0, 0, 1500);
    */

    //Go to the last neutral mogo and clamp it
    forwardJPIDfrontDistance(4, 0, 0, 1, 0.75, 0, 0, 4000);

    delay(250);

    frontClamp.set_value(false);




    /*
    //---------------------------------------------------------------------------------//
    //----------------------------------last section-----------------------------------//
    //---------------------------------------------------------------------------------//

    moveLift(2000, 1, 0, 0, 3000);

    basicForwardJPID(30, 3, 6, 0, 0, 2000);

    turnJPID2(225, 0, 1.3, 0, 0, 1500);
    turnJPID2(225, 0, 4, 0, 0, 1500);

    basicForwardJPID(8, 3, 6, 0, 0, 2000);

    turnJPID2(270, 0, 1.3, 0, 0, 1500);
    turnJPID2(270, 0, 4, 0, 0, 1500);

    moveLift(10000, 1, 0, 0, 3000);

    basicForwardJPID(40, 3, 6, 0, 0, 2000);

    basicForwardJPID(-5, 3, 6, 0, 0, 2000);

    turnJPID2(360, 0, 1.3, 0, 0, 1500);
    turnJPID2(360, 0, 4, 0, 0, 1500);

    basicForwardJPID(5, 3, 6, 0, 0, 2000);

    moveLift(0, 1, 0, 0, 3000);

    basicForwardJPID(15, 3, 6, 0, 0, 2000);

    balance(20, 15, 2000); 
    */

    // lift up the lift while turning to the right
    pros::Task lambdaTask10{[=] { moveLift(10000, 1, 0, 0, 3000); }};
    turnJPID2(270, 0, 4, 0, 0, 1500);
    // spin the conveyor 
    conveyor.move(127);
    // go towards some of the rings
    basicForwardJPID(15, 3, 6, 0, 0, 2000);
    // turn right
    turnJPID2(360, 0, 1.6, 0, 0, 1500);
    turnJPID2(360, 0, 8, 0, 0, 1000);
    // go forwards
    basicForwardJPID(42, 3, 6, 0, 0, 2000);

    // turn left
    turnJPID2(270, 0, 2, 0, 0, 1500);
    turnJPID2(270, 0, 12, 0, 0, 1000);
    // go forward a little bit
    basicForwardJPID(9.25, 3, 6, 0, 0, 1000);
    // move the lift down to balance the ramp (first mogo on the ramp)
    moveLift(6500, 1, 0, 0, 3000);

    delay(200);
    // unclamp 
    frontClamp.set_value(true);
    // go back a little bit 
    basicForwardJPID(-3, 3, 7.5, 0, 0, 1000);

    // move the lift up 
    moveLift(10000, 1, 0, 0, 3000);

    //finish going backwards
    basicForwardJPID(-4, 3, 7.5, 0, 0, 1000);
    pros::delay(250);

    // go back some more while putting the lift down
    pros::Task lambdaTask6{[=] { moveLift(0, 1, 0, 0, 3000); }};
    basicForwardJPID(-11, 3, 6, 0, 0, 1000); //used to be -13 but changed to prevent the robot from moving

    turnJPID2(280, 0, 1.3, 0, 0, 3000);
    turnJPID2(280, 0, 4, 0, 0, 1000);
    // unclamp the back 
    backClamp.set_value(false);
    // go forward a little bit
    basicForwardJPID(10, 3, 6, 0, 0, 1000); //used to be 10 but supposed to be 8 but made it 10 to appaoch more

    // turn to face the back mogo
    turnJPID2(100, 0, 1.3, 0, 0, 1000);
    turnJPID2(100, 0, 4, 0, 0, 1000);
    // go forward to clamp the mogo
    forwardJPIDfrontDistance(4, 0, 0, 1, 0.4, 0, 0, 1500);
    frontClamp.set_value(false);
    basicForwardJPID(-5, 3, 6, 0, 0, 1000);

    // turn to face the ramp while lifting up the mogo
    pros::Task lambdaTask7{[=] { moveLift(10000, 1, 0, 0, 3000); }};
    turnJPID2(280, 0, 1.3, 0, 0, 1500);
    turnJPID2(280, 0, 4, 0, 0, 1000);

    // go forward a little bit
    basicForwardJPID(20, 3, 6, 0, 0, 1000);
    // move the lift down to balance the ramp
    moveLift(6000, 1, 0, 0, 3000);
    // unclamp 
    frontClamp.set_value(true);

    basicForwardJPID(-10, 3, 6, 0, 0, 1000);

}