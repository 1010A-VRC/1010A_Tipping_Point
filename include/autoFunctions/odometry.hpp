/**
 * @file odometry.hpp
 * @author Liam Teale
 * @brief Header file containing definitions to functions related to position tracking
 * @version 0.1
 * @date 2022-01-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once


class odometry {

    public:
        
        odometry(pros::ADIEncoder* sideTrack, bool isRight, double sideDiameter, double sideDistance, pros::ADIEncoder *centerTrack, bool isBack, double centerDiameter, double centerDistance);
        double get_x();
        double get_y();
        double get_heading(bool radians);
        void start_tracking();
        void stop_tracking();
        void tracking();

    private:
        double filter(double prevValue, double curValue);
        pros::ADIEncoder* sideTrackWheel;
        bool isSideRight;
        double sideTrackDiameter;
        double sideTrackDistance;
        pros::ADIEncoder* centerTrackWheel;
        bool isCenterBack;
        double centerTrackDiameter;
        double centerTrackDistance;
        
        double x;
        double y;
        double h;

        double deg_to_rad(double deg);
        double rad_to_deg(double rad);

        bool runTracking;
        pros::Task* trackingThread;

        static void start_tracking_helper();

};


/**
 * @brief Instance of odometry
 * 
 */
extern odometry odom;