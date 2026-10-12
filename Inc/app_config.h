/**
 ******************************************************************************
 * @file    app_config.h
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#ifndef APP_CONFIG
#define APP_CONFIG

#define USE_DCACHE

/* Define sensor orientation */
#define CAMERA_FLIP CMW_MIRRORFLIP_MIRROR

/* Define display size */
#define LCD_BG_WIDTH  800
#define LCD_BG_HEIGHT 480
/* Delay display by DISPLAY_DELAY frame number */
#define DISPLAY_DELAY 1

/* Define NN pipe format */
#define NN_PIPE_FORMAT DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1
#define NN_PIPE_BPP 3
#define NN_PIPE_WIDTH  640
#define NN_PIPE_HEIGHT 480

/* Lost / warn detection thresholds */
#define PSR_LOST_THRESHOLD  8.0f
#define PSR_WARN_THRESHOLD 15.0f

#endif
