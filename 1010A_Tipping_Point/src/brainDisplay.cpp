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
#include "display/lv_objx/lv_btn.h"
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
 * @brief function that creates an animation
 * 
 * @param var Variable to animate
 * @param start Start value
 * @param end End value
 * @param fp Function to be used to animate
 * @param path Path of animation
 * @param end_cb Callback when the animation is ready
 * @param act_time Set < 0 to make a delay [ms]
 * @param time Animation length [ms]
 * @param playback 1: animate in reverse direction too when the normal is ready
 * @param playback_pause Wait before playback [ms]
 * @param repeat 1: Repeat the animation (with or without playback)
 * @param repeat_pause Wait before repeat [ms]
 * @return lv_anim_t 
 */
lv_anim_t lv_animation_creator(lv_obj_t * var, int32_t start, int32_t end, lv_anim_fp_t fp, lv_anim_path_t path, lv_anim_cb_t end_cb, int16_t act_time, uint16_t time, uint8_t playback, uint16_t playback_pause, uint8_t repeat, uint16_t repeat_pause) 
{
    /** create a new animation object */
    lv_anim_t animation;

    animation.var = var; /**< Variable to animate                                                    */
    animation.start = start; /**< Start value                                                        */
    animation.end = end; /**< End value                                                              */
    animation.fp = fp; /**< Function to be used to animate                                           */
    animation.path = path; /**< Path of animation                                                    */
    animation.end_cb = end_cb; /**< Callback when the animation is ready                             */
    animation.act_time = act_time; /**< Set < 0 to make a delay [ms]                                 */
    animation.time = time; /**< Animation length [ms]                                                */
    animation.playback = playback; /**< 1: animate in reverse direction too when the normal is ready */
    animation.playback_pause = playback_pause; /**< Wait before playback [ms]                        */
    animation.repeat = repeat; /**< 1: Repeat the animation (with or without playback)               */
    animation.repeat_pause = repeat_pause; /**< Wait before repeat [ms]                              */

    /** return the animation object */
    return animation;
}



/** objects used by LVGL */
lv_obj_t * wallpaper_img;
lv_obj_t * main_title_img;
lv_anim_t main_title_in;
lv_anim_t main_title_out;
lv_obj_t * auto_selector_btn;
lv_obj_t * debug_btn;
lv_obj_t * settings_btn;


/** what to do when the autonomous selector button is pressed */
static lv_res_t auto_selector_clicked(lv_obj_t * btn)
{
    /** make the main title go up */
    lv_anim_create(&main_title_out);
    lv_obj_del(auto_selector_btn);
    lv_obj_del(debug_btn);
    lv_obj_del(settings_btn);
    pros::Task deleteMainTitleImg{[=] { while (lv_anim_count_running()) {pros::delay(100);} lv_obj_del(main_title_img); master.rumble("-");}};
    

    return LV_RES_OK; /*Return OK if the button is not deleted*/
}


/** what to do when the autonomous selector button is pressed */
static lv_res_t debug_clicked(lv_obj_t * btn)
{
    /** make the main title go up */
    lv_anim_create(&main_title_out);
    lv_obj_del(auto_selector_btn);
    lv_obj_del(debug_btn);
    lv_obj_del(settings_btn);
    pros::Task deleteMainTitleImg{[=] { while (lv_anim_count_running()) {pros::delay(100);} lv_obj_del(main_title_img); master.rumble("-");}};
    

    return LV_RES_OK; /*Return OK if the button is not deleted*/
}


/** what to do when the autonomous selector button is pressed */
static lv_res_t settings_clicked(lv_obj_t * btn)
{
    /** make the main title go up */
    lv_anim_create(&main_title_out);
    lv_obj_del(auto_selector_btn);
    lv_obj_del(debug_btn);
    lv_obj_del(settings_btn);
    pros::Task deleteMainTitleImg{[=] { while (lv_anim_count_running()) {pros::delay(100);} lv_obj_del(main_title_img); master.rumble("-");}};
    

    return LV_RES_OK; /*Return OK if the button is not deleted*/
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

    /** create the background */
    wallpaper_img = lv_img_create(NULL, NULL);
    lv_img_set_src(wallpaper_img, "S:/usd/images/wallpaper.bin");
    lv_scr_load(wallpaper_img);

    /** create the main title image */
    main_title_img = lv_img_create(lv_scr_act(), NULL);
    lv_img_set_src(main_title_img, "S:/usd/images/title.bin");
    lv_obj_align(main_title_img, NULL, LV_ALIGN_OUT_TOP_MID, 0, 0);

    /** create animations for the main title */
    main_title_in = lv_animation_creator(main_title_img, -50, 0, (lv_anim_fp_t)lv_obj_set_y, lv_anim_path_ease_out, NULL, 0, 300, 0, 0, 0, 0); /**< function that animates the main title to appear */
    main_title_out = lv_animation_creator(main_title_img, 0, -50, (lv_anim_fp_t)lv_obj_set_y, lv_anim_path_ease_in, NULL, 0, 300, 0, 0, 0, 0); /**< function that animates the main title to appear */

    /** animate the main title */
    lv_anim_create(&main_title_in);
    pros::delay(1000);

    /** create a button that goes to the autonomous selector screen */
    auto_selector_btn = lv_imgbtn_create(lv_scr_act(), NULL);
    lv_imgbtn_set_src(auto_selector_btn, LV_BTN_STATE_REL, "S:/usd/images/autoREL.bin");
    lv_imgbtn_set_src(auto_selector_btn, LV_BTN_STATE_PR, "S:/usd/images/autoPR.bin");
    lv_obj_align(auto_selector_btn, NULL, LV_ALIGN_IN_LEFT_MID, 0, 0);
    lv_imgbtn_set_action(auto_selector_btn, LV_BTN_ACTION_CLICK, auto_selector_clicked);

    /** create a button that goes to the autonomous selector screen */
    debug_btn = lv_imgbtn_create(lv_scr_act(), NULL);
    lv_imgbtn_set_src(debug_btn, LV_BTN_STATE_REL, "S:/usd/images/debugREL.bin");
    lv_imgbtn_set_src(debug_btn, LV_BTN_STATE_PR, "S:/usd/images/debugPR.bin");
    lv_obj_align(debug_btn, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_imgbtn_set_action(debug_btn, LV_BTN_ACTION_CLICK, debug_clicked);

    /** create a button that goes to the autonomous selector screen */
    settings_btn = lv_imgbtn_create(lv_scr_act(), NULL);
    lv_imgbtn_set_src(settings_btn, LV_BTN_STATE_REL, "S:/usd/images/settingsREL.bin");
    lv_imgbtn_set_src(settings_btn, LV_BTN_STATE_PR, "S:/usd/images/settingsPR.bin");
    lv_obj_align(settings_btn, NULL, LV_ALIGN_IN_RIGHT_MID, 0, 0);
    lv_imgbtn_set_action(settings_btn, LV_BTN_ACTION_CLICK, settings_clicked);
    
    

}