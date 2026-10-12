/*---------------------------------------------------------------------------------------------
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *--------------------------------------------------------------------------------------------*/

#ifndef __VOT_LIGHTTRACKM_PP_IF_H__
#define __VOT_LIGHTTRACKM_PP_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"
#include "vot_pp_output.h"

/* Error return codes */
#define AI_VOT_LIGHTTRACKM_PP_ERROR_NO      ( 0)
#define AI_VOT_LIGHTTRACKM_PP_ERROR_BAD_HW  (-1) // Currently unused; reserved for future use
#define AI_VOT_LIGHTTRACKM_PP_ERROR         (-2)

/* Number of float32_t elements required in the p_scratch work buffer.
 * Use VOT_LIGHTTRACKM_PP_SCRATCH_SIZE(score_sz) when allocating the buffer.
 *
 * Layout (2 * score_size² covers both call variants):
 *   [0 .. N)    cls_sigmoid (process) / cls_float (process_is8)
 *   [N .. 2N)   pscore (both variants)
 */
#define VOT_LIGHTTRACKM_PP_SCRATCH_SIZE(n)  (2u * ((n) * (n)))


/* -------------------------------------------------------------------------
 * Input structure —
 * -------------------------------------------------------------------------
 * cls_raw : pointer to the raw classification output of the ONNX model,
 *           shape [1, 1, SCORE_SIZE, SCORE_SIZE] = 144 float32 values,
 *           stored in row-major order (NCHW, batch=1, C=1).
 *
 * reg_raw : pointer to the raw regression output of the ONNX model,
 *           shape [1, 4, SCORE_SIZE, SCORE_SIZE] = 576 float32 values,
 *           stored in row-major order (NCHW, batch=1, C=4).
 *           Channel order: [dx1, dy1, dx2, dy2]
 *           (distance from cell centre to left, top, right, bottom edges).
 * -------------------------------------------------------------------------
 */
typedef struct vot_lighttrackm_pp_in
{
    const void *cls_raw;   /* [1,1,SCORE_SIZE,SCORE_SIZE] = 144 floats — read-only                                   */
    const void *reg_raw;   /* [1,4,SCORE_SIZE,SCORE_SIZE] = 576 floats — read-only */
} vot_lighttrackm_pp_in_t;

/* -------------------------------------------------------------------------
 * Static parameters  (set once per tracking sequence, not per-frame)
 * -------------------------------------------------------------------------
 * target_pos_x           : current target centre x in image coordinates
 * target_pos_y           : current target centre y in image coordinates
 * target_sz_scaled[2]   : target size (w, h) multiplied by scale_z,
 *                         i.e. expressed in search-crop coordinates
 * scale_z               : TEMPLATE_SIZE / sqrt(wc_z * hc_z),
 *                         computed by the host tracking loop
 * penalty_k             : size/ratio penalty coefficient   (default 0.04)
 * window_influence      : Hanning window blend weight      (default 0.38)
 * lr                    : size smoothing learning rate     (default 0.82)
 * p_grid_x              : pointer to the pre-built GRID_X table [144 floats].
 * p_grid_y              : pointer to the pre-built GRID_Y table [144 floats].
 * p_window              : pointer to the [SCORE_SIZE x SCORE_SIZE] window
 *                         (float32, row-major).
 *                         - Normal tracking : point to the Hanning window
 *                         - Grid-search / redet : point to an all-ones buffer
 *                           of the same size (144 floats).
 * -------------------------------------------------------------------------
 */
typedef struct vot_lighttrackm_pp_static_param
{
    /* /!\ 5 following parameters are not static but may be updated between each run */
    float32_t  target_pos_x;           /* current target centre x, image coords    */
    float32_t  target_pos_y;           /* current target centre y, image coords    */
    float32_t  target_sz_scaled_w;     /* target size * scale_z  (search coords)   */
    float32_t  target_sz_scaled_h;     /* target size * scale_z  (search coords)   */
    float32_t  scale_z;                /* scale factor: TEMPLATE_SIZE / s_z        */
    /* End of not static parameters */
    uint32_t   score_size;             /* score map side length (e.g. 12)          */
    uint32_t   stride;                 /* feature stride (e.g. 16)                 */
    uint32_t   search_size;            /* search image size (e.g. 192)             */
    float32_t  penalty_k;              /* size/ratio penalty coefficient           */
    float32_t  window_influence;       /* Hanning window blend weight              */
    float32_t  lr;                     /* size smoothing learning rate             */
    const uint16_t  *p_grid_x;         /* [score_size*score_size] position grid X — integer pixel coords, provided by caller */
    const uint16_t  *p_grid_y;         /* [score_size*score_size] position grid Y — integer pixel coords, provided by caller */
    const float32_t *p_window;         /* [score_size*score_size] Hanning (or all-ones) window, provided by caller           */
    float32_t  cls_scale;              /* cls output dequant scale      */
    int8_t     cls_zero_point;         /* cls output dequant zero-point */
    float32_t  reg_scale;              /* reg output dequant scale      */
    int8_t     reg_zero_point;         /* reg output dequant zero-point */
    float32_t *p_scratch;              /* [VOT_LIGHTTRACKM_PP_SCRATCH_SIZE(score_size)] work buffer, caller-allocated */
} vot_lighttrackm_pp_static_param_t;


/* -------------------------------------------------------------------------
 * Exported functions
 * -------------------------------------------------------------------------
 */

/*!
 * @brief  Validate and initialise the vot_lighttrackm_pp_static_param_t structure.
 *
 *         Checks that all mandatory fields are set (non-NULL grid/window/scratch
 *         pointers, score_size > 3, non-zero stride and search_size). *
 *         The grid and window arrays themselves are NOT computed here — they must
 *         be pre-built by the caller (e.g. from Python-generated headers) and
 *         pointed to by p_grid_x, p_grid_y and p_window before calling this
 *         function.
 *
 *         Call once before the tracking loop.
 *
 * @param [IN/OUT] pStatic  Pointer on static parameters. On entry, p_grid_x,
 *                          p_grid_y and p_window must be non-NULL, and
 *                          score_size/stride/search_size must be non-zero.
 * @retval AI_VOT_LIGHTTRACKM_PP_ERROR_NO on success, negative on error.
 */
int32_t vot_lighttrackm_pp_reset(vot_lighttrackm_pp_static_param_t *pStatic);


/*!
 * @brief  Decode one frame of LightTrackM model outputs into a new target
 *         position and size.
 *
 *         Equivalent to tracker_decode() in camera_demo_PC.ipynb (Cell 2).
 *
 * @param [IN]  pInput   Pointer on raw model outputs (cls_raw, reg_raw).
 * @param [OUT] pOutput  Pointer on decoded result (vot_pp_out_t: new_pos, new_sz, best_score, psr).
 * @param [IN]  pStatic  Pointer on static parameters (must have been
 *                       initialised by vot_lighttrackm_pp_reset()).
 * @retval AI_VOT_LIGHTTRACKM_PP_ERROR_NO on success, negative on error.
 */
int32_t vot_lighttrackm_pp_process(const vot_lighttrackm_pp_in_t            *pInput,
                                   vot_pp_out_t                 *pOutput,
                                   const vot_lighttrackm_pp_static_param_t  *pStatic);


/*!
 * @brief  INT8 variant of vot_lighttrackm_pp_process().
 *
 *         Dequantizes the INT8 cls and reg model outputs to float32 using the
 *         parameters in pDequant, then runs the standard decode pipeline.
 *         Equivalent to tracker_decode_is8() in camera_demo_PC.ipynb.
 *
 * @param [IN]  pInput    Pointer on INT8 model outputs (cls_raw_is8, reg_raw_is8).
 * @param [OUT] pOutput   Pointer on decoded result (vot_pp_out_t, same layout as float variant).
 * @param [IN]  pStatic   Pointer on static parameters (initialised by
 *                        vot_lighttrackm_pp_reset(), shared with float variant).
 * @retval AI_VOT_LIGHTTRACKM_PP_ERROR_NO on success, negative on error.
 */
int32_t vot_lighttrackm_pp_process_is8(const vot_lighttrackm_pp_in_t         *pInput,
                                        vot_pp_out_t                  *pOutput,
                                        const vot_lighttrackm_pp_static_param_t   *pStatic);


#ifdef __cplusplus
}
#endif

#endif /* __VOT_LIGHTTRACKM_PP_IF_H__ */
