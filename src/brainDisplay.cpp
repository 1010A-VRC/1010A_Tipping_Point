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

#include "display/lv_core/lv_obj.h"
#include "display/lv_core/lv_style.h"
#include "display/lv_fonts/lv_font_builtin.h"
#include "display/lv_misc/lv_color.h"
#include "display/lv_objx/lv_btn.h"
#include "display/lv_objx/lv_label.h"
#include "main.h"
#include "robot_config.hpp"
#include "autoFunctions/odometry.hpp"
#include "brainDisplay.hpp"
#include "images.h"


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
 * @brief function using LVGL, a more 
 * 
 */
void brain_screen::brain_display() 
{
    /** set the style used for the background */
    static lv_style_t default_background;
    /** parameters for the background */
    lv_style_copy(&default_background, &lv_style_pretty_color);
    default_background.body.main_color = LV_COLOR_MAKE(0, 0, 0);
    default_background.body.grad_color = LV_COLOR_MAKE(0, 0, 0);
    /** apply the parameters for the background */
    lv_obj_set_style(lv_scr_act(), &default_background);
    lv_obj_refresh_style(lv_scr_act());

    /** create the main title (AMOGUS) */
    lv_obj_t *main_title = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_text(main_title, "AMOGUS\n");
    /** style the main title */
    static lv_style_t main_title_style;
    lv_style_copy(&main_title_style, &lv_style_plain);
    main_title_style.text.color = LV_COLOR_RED;
    main_title_style.text.font = &lv_font_dejavu_20;
    /** apply the style to the main title */
    lv_label_set_style(main_title, &main_title_style);

    /** declare main title image */
    LV_IMG_DECLARE(main_title_image);
    lv_obj_t * main_title_img = lv_img_create(lv_scr_act(), NULL);
    lv_img_set_src(main_title_img, &main_title_image);
    lv_obj_align(main_title_img, NULL, LV_ALIGN_IN_TOP_LEFT, 20, 40);


}