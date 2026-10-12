/*---------------------------------------------------------------------------------------------
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *--------------------------------------------------------------------------------------------*/

#ifndef __VOT_PP_OUTPUT_H__
#define __VOT_PP_OUTPUT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"

/* -------------------------------------------------------------------------
 * Generic visual object tracking post-processing output structure.
 * Shared by all VOT post-processing modules (e.g. LightTrackM).
 *
 * All coordinates and sizes are expressed in original image coordinates.
 * -------------------------------------------------------------------------
 */
typedef struct vot_pp_out
{
    float32_t x_center;     /* x centre of tracked object, image coordinates  */
    float32_t y_center;     /* y centre of tracked object, image coordinates  */
    float32_t width;        /* width  of tracked object, image coordinates    */
    float32_t height;       /* height of tracked object, image coordinates    */
    float32_t best_score;   /* cls sigmoid score at argmax(pscore)            */
    float32_t psr;          /* Peak-to-Sidelobe Ratio of the cls_score map    */
} vot_pp_out_t;


#ifdef __cplusplus
}
#endif

#endif /* __VOT_PP_OUTPUT_H__ */
