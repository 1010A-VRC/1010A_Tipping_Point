#pragma once

// data structure containing information of an object tracked by a vision sensor
typedef struct signature {
    double distance;
    double x;
    double y;
    double angle_from_robot;

} signature_data;


// functions
void moveLeftDrivetrain(double speed);
void moveRightDrivetrain(double speed);
void stopDrivetrain();
void forwardJPIDTrack(double goalX, double goalY, double kJ, double kP, double kI, double kD, double maxTime);
void forwardJPIDfrontDistance(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime);
void forwardJPIDbackDistance(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime);
void backwardJPIDTrack(double goalX, double goalY, double kJ, double kP, double kI, double kD, double maxTime);
void backwardJPIDbackDistance(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime);
void turnJPID(double goal, double kJ, double kP, double kI, double kD, double maxTime);
void forwardVisionTracking(int sigID, pros::vision_signature_s_t* sig, double turnKP);
void back_vision_align(int sigID, pros::vision_signature_s_t* sig, double turnKP, double turnKI, double turnKD, double maxTime);
void basicForwardJPID(double goal, double kJ, double kP, double kI, double kD, double maxTime);
void backwardJPIDbackDistance2(double goal, double expectedDistance, double clampOffset, double kJ, double kP, double kI, double kD, double maxTime);
void turnJPID2(double goal, double kJ, double kP, double kI, double kD, double maxTime);

