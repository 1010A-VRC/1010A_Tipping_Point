/**
 * @file odometry.cpp
 * @author Liam Teale
 * @brief Function containing functions relating to position tracking
 * @version 0.1
 * @date 2022-01-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "main.h"
#include "robot_config.hpp"
#include "dataManagement/fileIO.hpp"
#include "autoFunctions/odometry.hpp"



/**
 * @brief Construct a new odometry::odometry object
 * 
 * @param sideTrack side tracking wheel object
 * @param isRight if the side tracking wheel is on the right or not
 * @param sideDiameter the diameter of the side tracking wheel
 * @param sideDistance the left-right distance between the tracking center and the side tracking wheel
 * @param centerTrack center tracking wheel object
 * @param isBack if the center tracking wheel is at the back or not
 * @param centerDiameter the diameter of the center tracking wheel
 * @param centerDistance // the forward-backward distance between the tracking center and the center tracking wheel
 *
 */
odometry::odometry(pros::ADIEncoder* sideTrack, bool isRight, double sideDiameter, double sideDistance, pros::ADIEncoder* centerTrack, bool isBack, double centerDiameter, double centerDistance) 
{
    /** set tracking wheel pointers */
    odometry::sideTrackWheel = sideTrack;
    odometry::centerTrackWheel = centerTrack;

    /** save which side the tracking wheels are located on */
    odometry::isSideRight = isRight;
    odometry::isCenterBack = isBack;

    /** if the side tracking wheel is right, flip the measurement sign */
    if (isRight) {
        odometry::sideTrackDistance = -sideDistance;
    /** else don't flip its measurement sign */
    } else {
        odometry::sideTrackDistance = sideDistance;
    }

    /** if the center traking wheel is at the back, flip the measurement sign */
    if (isBack) {
        odometry::centerTrackDistance = -centerDistance;
    /** else dont flip its measurement sign */
    } else {
        odometry::centerTrackDistance = centerDistance;
    }

    /** save the diameter of the wheels */
    odometry::sideTrackDiameter = sideDiameter;
    odometry::centerTrackDiameter = centerDiameter;
}


/**
 * @brief Function that returns the x position of the robot
 * 
 * @return double 
 */
double odometry::get_x() 
{
    return x;
}


/**
 * @brief Function that returns the y position of the robot
 * 
 * @return double 
 */
double odometry::get_y() 
{
    return y;
}


/**
 * @brief Function that returns the heading of the robot
 * 
 * @param radians 
 * @return double 
 */
double odometry::get_heading(bool radians) 
{
    if (radians) {
        return h; /**< return radians if radians == true */

    /** else return the heading of the robot in degrees, bounded by 0 to 360 */
    } else {
        /** if the total rotation of the robot is positive */
        if (h > 0) {
            return std::fmod(h*(180/M_PI), 360); /**< return degrees */

        /** else if the total rotation of the robot is negative */
        } else if (h < 0) {
            return std::fmod(h*(180/M_PI), 360) + 360; /**< return degrees, adds 360 to make the result positive */

        /** else if = equal 0, return 0 */
        } else {
            return 0;
        }
    }
}


/**
 * @brief Helper thread that starts the main tracking thread
 * 
 */
void odometry::start_tracking_helper() 
{
   odom.tracking();
}


/**
 * @brief Function that runs the main tracking thread
 * 
 */
void odometry::start_tracking() 
{   
    /** set the variable that allows the main function to run to true */
    runTracking = true;
    /** delay until both inertial sensors are done calibrating */
    while (imu1.is_calibrating() || imu2.is_calibrating()) {
        pros::delay(10);
    }
    /** schedule the helper thread in RTOS */
    trackingThread = new pros::Task(odometry::start_tracking_helper);
}


/**
 * @brief Stops the tracking thread
 * 
 */
void odometry::stop_tracking() 
{
    runTracking = false; /**< set the loop conditional to false                    */
    pros::delay(15); /**< delay to prevent RTOS freezing                           */
    trackingThread->remove(); /**< remove the thread from the RTOS scheduler       */
    delete trackingThread; /**< delete the trackingThread pointer's assigned value */
    trackingThread = nullptr; /** set trackingThread to a nullptr                  */
}


/**
 * @brief Function that filters the change in a value to prevent fluctuation
 * 
 */
double odometry::filter(double prevValue, double curValue) 
{
    /** if the change in value is less than 0.0025, return no change */
    if (std::fabs(curValue - prevValue) < 0.0025) {
        return 0;

    /** else return the change */
    } else {
        return curValue - prevValue;
    }
}


/**
 * @brief Function that converts degrees to radians
 * 
 * @param deg 
 * @return double 
 */ 
double odometry::deg_to_rad(double deg) {
    return deg * (M_PI/180);
}


/**
 * @brief Function that converts radians to degrees
 * 
 * @param rad 
 * @return double 
 */
double odometry::rad_to_deg(double rad) {
    return rad * (180/M_PI);
}



/**
 * @brief Main tracking function
 * 
 */
void odometry::tracking() 
{
    /** initialize variables */
    double localX = get_x(); /**< get starting x position                                                 */
    double localY = get_y(); /**< get starting y position                                                 */
    double localH = get_heading(true); /**< get starting heading                                          */
    double sideValue = 0; /**< the current value of the side tracking wheel, in degrees                   */
    double prevSideValue = 0; /**< the value of sideValue at the last cycle                               */
    double centerValue = 0; /**< the current value of the center tracking wheel, in degrees               */
    double prevCenterValue = 0; /**< the value of centerValue at the last cycle                           */
    double polarRadius = 0; /**< the radius when polar coordinates are being used                         */
    double polarTheta = 0; /**< the angle when polar coordinates are being used                           */
    double sideChord = 0; /**< the chord of the arc travelled by the side tracking wheel, in inches       */
    double centerChord = 0; /**< the chord of the arc travelled by the center tracking wheel, in inches   */
    double sideArc = 0; /**< the length of the arc travelled by the side tracking wheel, in inches        */
    double centerArc = 0; /**< the length of the arc travelled by the center tracking wheel, in inches    */
    double deltaTheta = 0; /**< the change in angle, in radians                                           */
    double averageTheta = 0; /**< the average heading of the robot when it moved along an arc, in radians */
    
    /** main loop, run while runTracking == true */
    while (runTracking) {

        /** update arcs travelled by the tracking wheels */
        sideValue = sideTrackWheel->get_value()*(M_PI*sideTrackDiameter/360); /**< get the total distance travelled by the side tracking wheel */
        centerValue = centerTrackWheel->get_value()*(M_PI*centerTrackDiameter/360); /**< get the total distance travelled by the center tracking wheel */

        /** get the distance travelled by the tracking wheels since last loop */
        sideArc = sideValue - prevSideValue; /**< arc travelled by side tracking wheel         */
        centerArc = centerValue - prevCenterValue; /**< arc travelled by center tracking wheel */

        prevSideValue = sideValue;
        prevCenterValue = centerValue;

        /** perform updates on the heading of the robot */
        deltaTheta = odometry::filter(localH, deg_to_rad((imu1.get_rotation() + imu2.get_rotation())/2)); /**< get the change in heading since the last rotation and filter it */
        averageTheta = localH + deltaTheta/2; /**< the average heading of the robot while it travelled along the arc                                                           */

        /** calculate the chord lengths of the tracking wheels arcs, and then change the arc radius so it is aligned with the center of the robot */
        if (deltaTheta != 0) { /**< if deltaTheta is not 0 */
            sideChord = 2*sin(fabs(deltaTheta)/2)*(sideArc/fabs(deltaTheta) + odometry::sideTrackDistance); /**< chord length of the side tracking wheel       */
            centerChord = 2*sin(fabs(deltaTheta)/2)*(centerArc/fabs(deltaTheta) + odometry::centerTrackDistance); /**< chord length of the side tracking wheel */
        } else {
            sideChord = sideArc;
            centerChord = centerArc;
        }
        
        /** calculate the local polar coordinates of the robot by converting the chord lengths (x and y values) to polar coordinates (radius, theta) */
        polarRadius = sqrt(pow(centerChord, 2) + pow(sideChord, 2)); /**< calculate the radius by using the pythagorean theorem */
        polarTheta = atan2(sideChord, centerChord); /**< calculate theta of the local polar coordinates                           */

        if (std::isnan(polarRadius)) {
            polarRadius = 0;
        }

        if (std::isnan(polarTheta)) {
            polarTheta = 0;
        }

        /** rotate the local polar coordinates by the average heading of the robot (polar coordinates angle increases counterclocwise) */
        polarTheta -= averageTheta;
    
        /** offset the global coordinates by the newly calculated polar coordinates by converting the polar coordinates back into cartesian coordinates */
        localX += polarRadius * cos(polarTheta); /**< x */
        localY += polarRadius * sin(polarTheta); /**< y */
        localH += deltaTheta; /**< heading              */

        /** update class variables */
        odometry::x = localX;
        odometry::y = localY;
        odometry::h = localH;

        /** delay so RTOS does not freeze */
        pros::delay(10);
    }
}