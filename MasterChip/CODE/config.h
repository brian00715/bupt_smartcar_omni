/**
 * @file config.h
 * @author simon
 * @brief 参数配置与宏定义等
 * @version 0.1
 * @date 2021-04-17
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef CONFIG_H_
#define CONFIG_H_

#define UNUSED(x) (void)(x) // 用于避免GCC变量不使用的警告

// ======================物理参数=========================
#define MAX_ROTATE_VEL (0.5f)
#define WHEEL_FRONT2BACK (0.195f) // m
#define WHEEL_LEFT2RIGHT (0.1745f)
#define DRIVE_WHEEL_RADIUS (0.0275f) //m
#define DRIVE_WHEEL_MAX_SPEED (0.5f)
#define DRIVE_WHEEL_MIN_SPEED (0.1f)

#endif
