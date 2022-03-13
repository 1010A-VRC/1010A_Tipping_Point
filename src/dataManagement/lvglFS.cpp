#include "main.h"
#include "dataManagement/lvglFS.hpp"


/**
 * @brief function that LVGL uses to open files
 * 
 * @param file_p pointer to the file
 * @param fn 
 * @param mode open the file for read or write
 * @return lv_fs_res_t 
 */
static lv_fs_res_t pcfs_open( void * file_p, const char * fn, lv_fs_mode_t mode)
{
    errno = 0;
    const char * flags = "";
    if(mode == LV_FS_MODE_WR) flags = "wb";
    else if(mode == LV_FS_MODE_RD) flags = "rb";
    else if(mode == (LV_FS_MODE_WR | LV_FS_MODE_RD)) flags = "a+";

    char buf[256];
    sprintf(buf, "/%s", fn);
    pc_file_t f = fopen(buf, flags);

    if(f == NULL)
      return LV_FS_RES_UNKNOWN;
    else {
      fseek(f, 0, SEEK_SET);
      pc_file_t * fp = (pc_file_t *)file_p;
      *fp = f;
    }

    return LV_FS_RES_OK;
}


/**
 * @brief function for LVGL to close a file
 * 
 * @param file_p pointer to the file
 * @return lv_fs_res_t 
 */
static lv_fs_res_t pcfs_close( void * file_p)
{
    pc_file_t * fp = (pc_file_t *)file_p;
    fclose(*fp);
    return LV_FS_RES_OK;
}


/**
 * @brief function that LVGL uses to read from a file
 * 
 * @param file_p pointer to the file
 * @param buf
 * @param btr 
 * @param br 
 * @return lv_fs_res_t 
 */
static lv_fs_res_t pcfs_read( void * file_p, void * buf, uint32_t btr, uint32_t * br)
{
    pc_file_t * fp =  (pc_file_t *)file_p;
    *br = fread(buf, 1, btr, *fp);
    return LV_FS_RES_OK;
}


/**
 * @brief function for the LVGL filesystem to seek
 * 
 * @param file_p pointer to the file
 * @param pos 
 * @return lv_fs_res_t 
 */
static lv_fs_res_t pcfs_seek( void * file_p, uint32_t pos)
{
    pc_file_t * fp = (pc_file_t *)file_p;
    fseek(*fp, pos, SEEK_SET);
    return LV_FS_RES_OK;
}


/**
 * @brief function that LVGL uses to tell
 * 
 * @param file_p file pointer
 * @param pos_p 
 * @return lv_fs_res_t 
 */
static lv_fs_res_t pcfs_tell( void * file_p, uint32_t * pos_p)
{
    pc_file_t * fp =  (pc_file_t *)file_p;
    *pos_p = ftell(*fp);
    return LV_FS_RES_OK;
}


/**
 * @brief function that initializes the LVGL filsystem
 * 
 * @param fs_drv lvgl filesystem driver object
 */
void lvgl_filsystem_init(lv_fs_drv_t* fs_drv) 
{
    memset(fs_drv, 0, sizeof(lv_fs_drv_t)); /**< Initialization */
    /** Set up fields */
    fs_drv->file_size = sizeof(pc_file_t);       
    fs_drv->letter = 'S';
    fs_drv->open = pcfs_open;
    fs_drv->close = pcfs_close;
    fs_drv->read = pcfs_read;
    fs_drv->seek = pcfs_seek;
    fs_drv->tell = pcfs_tell;
    lv_fs_add_drv(fs_drv);
}