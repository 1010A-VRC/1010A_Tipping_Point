#pragma once 

/**
 * @brief pointer to a file opened by LVGL
 * 
 */
typedef  FILE * pc_file_t;


/**
 * @brief function that LVGL uses to open files
 * 
 * @param file_p pointer to the file
 * @param fn 
 * @param mode open the file for read or write
 * @return lv_fs_res_t 
 */
static lv_fs_res_t pcfs_open( void * file_p, const char * fn, lv_fs_mode_t mode);


/**
 * @brief function for LVGL to close a file
 * 
 * @param file_p pointer to the file
 * @return lv_fs_res_t 
 */
static lv_fs_res_t pcfs_close( void * file_p);


/**
 * @brief function that LVGL uses to read from a file
 * 
 * @param file_p pointer to the file
 * @param buf
 * @param btr 
 * @param br 
 * @return lv_fs_res_t 
 */
static lv_fs_res_t pcfs_read( void * file_p, void * buf, uint32_t btr, uint32_t * br);


/**
 * @brief function for the LVGL filesystem to seek
 * 
 * @param file_p pointer to the file
 * @param pos 
 * @return lv_fs_res_t 
 */
static lv_fs_res_t pcfs_seek( void * file_p, uint32_t pos);


/**
 * @brief function that LVGL uses to tell
 * 
 * @param file_p file pointer
 * @param pos_p 
 * @return lv_fs_res_t 
 */
static lv_fs_res_t pcfs_tell( void * file_p, uint32_t * pos_p);


/**
 * @brief function that initializes the LVGL filsystem
 * 
 * @param fs_drv lvgl filesystem driver object
 */
void lvgl_filsystem_init(lv_fs_drv_t* fs_drv);