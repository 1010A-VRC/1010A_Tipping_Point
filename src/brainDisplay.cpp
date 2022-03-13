/**
 * @file brainDisplay.cpp
 * @author Liam Teale
 * @brief File containing functions for the brain screen display
 * @version 0.1
 * @date 2022-01-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "display/lv_misc/lv_anim.h"
#include "main.h"
#include "robot_config.hpp"
#include "autoFunctions/odometry.hpp"
#include "brainDisplay.hpp"
#include "dataManagement/lvglFS.hpp"

/**
 * @brief function that returns the average temperature of 3 motors
 * 
 * @param a motor a 
 * @param b motor b
 * @param c motor c
 * @return double 
 */
double brain_screen::average_temperature(pros::Motor *a, pros::Motor *b, pros::Motor *c) 
{
    return (a->get_temperature() + b->get_temperature() + c->get_temperature())/3;
}


/**
 * @brief function that returns the average wattage of 3 motors
 * 
 * @param a 
 * @param b 
 * @param c 
 * @return double 
 */
double brain_screen::average_wattage(pros::Motor *a, pros::Motor *b, pros::Motor *c) 
{
    return (a->get_power() + b->get_power() + c->get_power())/3;
}


/**
 * @brief functions using LLEMU, the legacy LED emulator
 * 
 */
void brain_screen::legacy_brain_display() 
{

    std::string a, b, c;
    /** loop forever */

    // imu1 pitch: imu1.get_roll()
    // imu2 pitch: -imu2.get_roll()

    while (true) {

        pros::lcd::print(0, "X: %f", odom.get_x());
        pros::lcd::print(1, "Y: %f", odom.get_y());
        pros::lcd::print(2, "H: %f", odom.get_heading(false));
        pros::lcd::print(3, "Left side Temperature: %f", brain_screen::average_temperature(&l1, &l2, &l3)); /**< print the left drivetrain motor temperatures   */
        pros::lcd::print(4, "Right Side Temperature: %f", brain_screen::average_temperature(&r1, &r2, &r3)); /**< print the right drivetrain motor temperatures */
        pros::lcd::print(5, "Conveyor Temperature: %f", conveyor.get_temperature()); /**< print the temperature of the conveyor motor             */
        pros::lcd::print(6, "Lift temperature: %f", lift.get_temperature()); /**< print the temperature of the lift motor                         */
        pros::lcd::print(7, "pitch: %f", (imu1.get_roll() + -imu2.get_roll())/2);
        /** delay to prevent RTOS freezing */
        pros::delay(15);
    }
}


/**
 * @brief function using LVGL, a more comprehensive graphics library
 * 
 */
void brain_screen::brain_display() 
{
    /** initialize filesystem drivers */
    lv_fs_drv_t pcfs_drv; /**< A driver descriptor                 */
    lvgl_filsystem_init(&pcfs_drv);

    /** set the style used for the background */
    static lv_style_t default_background;
    /** parameters for the background */
    lv_style_copy(&default_background, &lv_style_pretty_color);
    default_background.body.main_color = LV_COLOR_MAKE(0, 0, 0);
    default_background.body.grad_color = LV_COLOR_MAKE(0, 0, 0);
    /** apply the parameters for the background */
    lv_obj_set_style(lv_scr_act(), &default_background);
    lv_obj_refresh_style(lv_scr_act());

    /** create the main title image */
    lv_obj_t * main_title_img = lv_img_create(lv_scr_act(), NULL);
    lv_img_set_src(main_title_img, "S:/usd/images/title.bin");
    lv_obj_align(main_title_img, NULL, LV_ALIGN_OUT_TOP_MID, 0, 0);

    /** animate the main title */
    lv_anim_t main_title_in_animation;
    main_title_in_animation.var = main_title_img; /**< Variable to animate                                         */
    main_title_in_animation.start = -40; /**< Start value                                                   */
    main_title_in_animation.end = 0; /**< End value                                                       */
    main_title_in_animation.fp = (lv_anim_fp_t)lv_obj_set_y; /**< Function to be used to animate       */
    main_title_in_animation.path = lv_anim_path_ease_out; /**< Path of animation                              */
    main_title_in_animation.end_cb = NULL; /**< Callback when the animation is ready                        */
    main_title_in_animation.act_time = 0; /**< Set < 0 to make a delay [ms]                                 */
    main_title_in_animation.time = 500; /**< Animation length [ms]                                          */
    main_title_in_animation.playback = 0; /**< 1: animate in reverse direction too when the normal is ready */
    main_title_in_animation.playback_pause = 0; /**< Wait before playback [ms]                              */
    main_title_in_animation.repeat = 0; /**< 1: Repeat the animation (with or without playback)             */
    main_title_in_animation.repeat_pause = 0; /**< Wait before repeat [ms]                                  */
    lv_anim_create(&main_title_in_animation); /**< Start the animation                                      */
}