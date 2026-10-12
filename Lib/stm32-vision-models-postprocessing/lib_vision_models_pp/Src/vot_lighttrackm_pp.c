/*---------------------------------------------------------------------------------------------
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *--------------------------------------------------------------------------------------------*/

#include "vot_lighttrackm_pp_if.h"
#include "vot_pp_loc.h"
#include "vision_models_pp.h"

/* ----------------------       Internal helpers      ---------------------- */

/* Effective scale-aware size: sqrt((w + pad) * (h + pad)), pad = (w + h) / 2 */
static inline float32_t _vot_sz(float32_t w, float32_t h)
{
    float32_t pad = (w + h) * 0.5f;
    return sqrtf((w + pad) * (h + pad));
}

/* Symmetrises a ratio: max(r, 1/r) */
static inline float32_t _vot_change(float32_t r)
{
    float32_t inv_r = 1.0f / r;
    return (r > inv_r) ? r : inv_r;
}

/* Target box aspect ratio and inverse effective size, in search-crop coordinates */
static inline void _vot_target_shape(const vot_lighttrackm_pp_static_param_t *pStatic,
                                      float32_t *p_ratio_target, float32_t *p_inv_sz_target)
{
    const float32_t tsz_w = pStatic->target_sz_scaled_w;
    const float32_t tsz_h = pStatic->target_sz_scaled_h;
    *p_ratio_target   = tsz_w / tsz_h;
    *p_inv_sz_target  = 1.0f / _vot_sz(tsz_w, tsz_h);
}

/* Decodes a (xs, ys, w, h) box from grid anchor + 4 regression offsets */
static inline void _vot_decode_box(uint16_t grid_x, uint16_t grid_y,
                                    float32_t r0, float32_t r1, float32_t r2, float32_t r3,
                                    float32_t *p_xs, float32_t *p_ys,
                                    float32_t *p_w, float32_t *p_h)
{
    const float32_t x1 = (float32_t)grid_x - r0;
    const float32_t y1 = (float32_t)grid_y - r1;
    const float32_t x2 = (float32_t)grid_x + r2;
    const float32_t y2 = (float32_t)grid_y + r3;

    *p_xs = (x1 + x2) * 0.5f;
    *p_ys = (y1 + y2) * 0.5f;
    *p_w  = x2 - x1;
    *p_h  = y2 - y1;
}

/* Shape-change penalty between a candidate box (pw, ph) and the target box */
static inline float32_t _vot_penalty(float32_t pw, float32_t ph, float32_t ratio_target,
                                      float32_t inv_sz_target, float32_t penalty_k)
{
    float32_t pen = 0.0f;
    if (pw > 0.0f && ph > 0.0f) {
        const float32_t s_c = _vot_change(_vot_sz(pw, ph) * inv_sz_target);
        const float32_t r_c = _vot_change(ratio_target / (pw / ph));
        pen = expf(-(r_c * s_c - 1.0f) * penalty_k);
    }
    return pen;
}

/* Peak-to-sidelobe ratio of the sigmoid cls map, excluding the 3x3 peak window */
static float32_t _vot_psr(float32_t *cls, int32_t sz, uint32_t N)
{
    uint32_t  cls_best_idx_u;
    float32_t cls_best_val;
    vision_models_maxi_if32ou32(cls, N, &cls_best_val, &cls_best_idx_u);
    const int32_t cls_best_idx = (int32_t)cls_best_idx_u;

    const int32_t r_psr = cls_best_idx / sz;
    const int32_t c_psr = cls_best_idx % sz;
    const int32_t r_lo  = (r_psr > 0)      ? r_psr - 1 : 0;
    const int32_t r_hi  = (r_psr < sz - 1) ? r_psr + 1 : sz - 1;
    const int32_t c_lo  = (c_psr > 0)      ? c_psr - 1 : 0;
    const int32_t c_hi  = (c_psr < sz - 1) ? c_psr + 1 : sz - 1;

    const int32_t   sl_count = (int32_t)N - (r_hi - r_lo + 1) * (c_hi - c_lo + 1);
    const float32_t inv_n = 1.0f / (float32_t)sl_count;
    float32_t sl_sum = 0.0f, sl_sum2 = 0.0f;
    for (int32_t r = 0; r < r_lo; r++) {
        const float32_t *row = &cls[r * sz];
        for (int32_t c = 0; c < sz; c++) { const float32_t v = row[c]; sl_sum += v; sl_sum2 += v * v; }
    }
    for (int32_t r = r_lo; r <= r_hi; r++) {
        const float32_t *row = &cls[r * sz];
        for (int32_t c = 0;        c < c_lo; c++) { const float32_t v = row[c]; sl_sum += v; sl_sum2 += v * v; }
        for (int32_t c = c_hi + 1; c < sz;   c++) { const float32_t v = row[c]; sl_sum += v; sl_sum2 += v * v; }
    }
    for (int32_t r = r_hi + 1; r < sz; r++) {
        const float32_t *row = &cls[r * sz];
        for (int32_t c = 0; c < sz; c++) { const float32_t v = row[c]; sl_sum += v; sl_sum2 += v * v; }
    }
    const float32_t mean = sl_sum  * inv_n;
    const float32_t var  = sl_sum2 * inv_n - mean * mean;

    return (cls_best_val - mean) / (sqrtf((var > 0.0f) ? var : 0.0f) + 1e-6f);
}

/* Converts a decoded box from search-crop space to image space, blends it with
 * the previous target size and fills the output structure */
static void _vot_finalize_output(const vot_lighttrackm_pp_static_param_t *pStatic,
                                  float32_t pred_xs, float32_t pred_ys,
                                  float32_t pred_w, float32_t pred_h,
                                  float32_t pw_best, float32_t ph_best,
                                  float32_t ratio_target, float32_t inv_sz_target,
                                  float32_t best_score, float32_t psr,
                                  vot_pp_out_t *pOutput)
{
    const float32_t scale_z     = pStatic->scale_z;
    const float32_t search_half = (float32_t)(pStatic->search_size / 2u);
    const float32_t inv_scale_z = 1.0f / scale_z;

    const float32_t diff_xs = (pred_xs - search_half) * inv_scale_z;
    const float32_t diff_ys = (pred_ys - search_half) * inv_scale_z;
    pred_w *= inv_scale_z;
    pred_h *= inv_scale_z;

    const float32_t tsz_w = pStatic->target_sz_scaled_w;
    const float32_t tsz_h = pStatic->target_sz_scaled_h;
    const float32_t target_sz_orig_w = tsz_w * inv_scale_z;
    const float32_t target_sz_orig_h = tsz_h * inv_scale_z;

    const float32_t penalty_best = _vot_penalty(pw_best, ph_best, ratio_target,
                                                 inv_sz_target, pStatic->penalty_k);

    const float32_t lr      = penalty_best * best_score * pStatic->lr;
    const float32_t one_mlr = 1.0f - lr;

    const float32_t res_w = pred_w * lr + target_sz_orig_w * one_mlr;
    const float32_t res_h = pred_h * lr + target_sz_orig_h * one_mlr;

    pOutput->x_center   = pStatic->target_pos_x + diff_xs;
    pOutput->y_center   = pStatic->target_pos_y + diff_ys;
    pOutput->width      = res_w * lr + target_sz_orig_w * one_mlr;
    pOutput->height     = res_h * lr + target_sz_orig_h * one_mlr;
    pOutput->best_score = best_score;
    pOutput->psr        = psr;
}


/* ----------------------       Exported routines      ---------------------- */

int32_t vot_lighttrackm_pp_reset(vot_lighttrackm_pp_static_param_t *pStatic)
{
    if (!pStatic || !pStatic->p_scratch ||
        !pStatic->p_grid_x || !pStatic->p_grid_y || !pStatic->p_window ||
        pStatic->score_size <= 3u || pStatic->stride == 0u || pStatic->search_size == 0u)
    {
        return AI_VOT_LIGHTTRACKM_PP_ERROR;
    }


    return AI_VOT_LIGHTTRACKM_PP_ERROR_NO;
}


int32_t vot_lighttrackm_pp_process(const vot_lighttrackm_pp_in_t            *pInput,
                                   vot_pp_out_t                             *pOutput,
                                   const vot_lighttrackm_pp_static_param_t  *pStatic)
{
    if (!pInput || !pOutput || !pStatic || !pInput->cls_raw || !pInput->reg_raw ||
        !pStatic->p_grid_x || !pStatic->p_grid_y ||
        !pStatic->p_window || !pStatic->p_scratch ||
        pStatic->score_size <= 3u)
    {
        return AI_VOT_LIGHTTRACKM_PP_ERROR;
    }

    const uint16_t  *p_grid_x = pStatic->p_grid_x;
    const uint16_t  *p_grid_y = pStatic->p_grid_y;

    const uint32_t N  = pStatic->score_size * pStatic->score_size;
    const int32_t sz = (int32_t)pStatic->score_size;

    const float32_t penalty_k         = pStatic->penalty_k;
    const float32_t window_influence  = pStatic->window_influence;

    /* Target shape in search-crop coordinates */
    float32_t ratio_target, inv_sz_target;
    _vot_target_shape(pStatic, &ratio_target, &inv_sz_target);

    /* Fused pass: sigmoid + penalty + pscore blend */
    const float32_t * __restrict cls_raw_f  = (const float32_t *)pInput->cls_raw;
    float32_t * __restrict const cls_sigma  = pStatic->p_scratch;      /* scratch[0..N) */
    float32_t * __restrict const pscore     = pStatic->p_scratch + N;  /* scratch[N..2N) */

    const float32_t *reg_ch0 = (const float32_t *)(pInput->reg_raw);
    const float32_t *reg_ch1 = reg_ch0 +     N;
    const float32_t *reg_ch2 = reg_ch0 + 2 * N;
    const float32_t *reg_ch3 = reg_ch0 + 3 * N;

    const float32_t wi      = window_influence;
    const float32_t one_mwi = 1.0f - wi;
    const float32_t *p_win  = pStatic->p_window;

    for (int32_t i = 0; i < (int32_t)N; i++)
    {
        const float32_t cs = vision_models_sigmoid_f(cls_raw_f[i]);
        cls_sigma[i] = cs;
        const float32_t pw = reg_ch0[i] + reg_ch2[i];
        const float32_t ph = reg_ch1[i] + reg_ch3[i];
        const float32_t pen = _vot_penalty(pw, ph, ratio_target, inv_sz_target, penalty_k);
        pscore[i] = pen * cs * one_mwi + p_win[i] * wi;
    }

    /* argmax(pscore) */
    uint32_t  best_idx_u;
    float32_t best_val_dummy;
    vision_models_maxi_if32ou32(pscore, (uint32_t)N, &best_val_dummy, &best_idx_u);
    int32_t best_idx = (int32_t)best_idx_u;

    /* PSR: peak-to-sidelobe ratio on sigmoid cls map, 3x3 peak excluded */
    const float32_t psr = _vot_psr(cls_sigma, sz, N);

    /* Decode box at best cell in search-crop space */
    float32_t pred_xs, pred_ys, pred_w, pred_h;
    _vot_decode_box(p_grid_x[best_idx], p_grid_y[best_idx],
                    reg_ch0[best_idx], reg_ch1[best_idx], reg_ch2[best_idx], reg_ch3[best_idx],
                    &pred_xs, &pred_ys, &pred_w, &pred_h);

    /* Convert search-crop space -> image coordinates, blend and assemble output */
    _vot_finalize_output(pStatic, pred_xs, pred_ys, pred_w, pred_h, pred_w, pred_h,
                          ratio_target, inv_sz_target, cls_sigma[best_idx], psr, pOutput);

    return AI_VOT_LIGHTTRACKM_PP_ERROR_NO;
}


int32_t vot_lighttrackm_pp_process_is8(const vot_lighttrackm_pp_in_t            *pInput,
                                        vot_pp_out_t                            *pOutput,
                                        const vot_lighttrackm_pp_static_param_t *pStatic)
{
    if (!pInput || !pOutput || !pStatic || !pStatic->p_grid_x || !pStatic->p_grid_y ||
        !pStatic->p_window || !pInput->cls_raw || !pInput->reg_raw ||
        !pStatic->p_scratch ||
        pStatic->score_size <= 3u)
    {
        return AI_VOT_LIGHTTRACKM_PP_ERROR;
    }

    const uint32_t N  = pStatic->score_size * pStatic->score_size;
    const int32_t sz = (int32_t)pStatic->score_size;

    float32_t * __restrict const cls_float = pStatic->p_scratch;
    float32_t * __restrict const pscore    = pStatic->p_scratch + N;

    /* INT8 reg channel pointers */
    const int8_t * const reg_i8     = (const int8_t *)pInput->reg_raw;
    const int8_t * const reg_ch0_i8 = reg_i8;
    const int8_t * const reg_ch1_i8 = reg_i8 +     N;
    const int8_t * const reg_ch2_i8 = reg_i8 + 2 * N;
    const int8_t * const reg_ch3_i8 = reg_i8 + 3 * N;

    const float32_t reg_scale     = pStatic->reg_scale;
    const float32_t reg_2zp_scale = 2.0f * (float32_t)pStatic->reg_zero_point * reg_scale;

    /* Dequantise + sigmoid cls [N int8 -> float32] */
    {
        const float32_t cls_scale = pStatic->cls_scale;
        const float32_t cls_zp    = (float32_t)pStatic->cls_zero_point;
        const int8_t   *src       = (const int8_t *)pInput->cls_raw;
        for (int32_t i = 0; i < (int32_t)N; i++)
        {
            cls_float[i] = vision_models_sigmoid_f(((float32_t)src[i] - cls_zp) * cls_scale);
        }
    }

    /* Fused pass: INT8 reg pairs -> penalty + pscore blend */
    const float32_t penalty_k    = pStatic->penalty_k;
    const float32_t wi           = pStatic->window_influence;
    const float32_t one_mwi      = 1.0f - wi;
    float32_t ratio_target, inv_sz_target;
    _vot_target_shape(pStatic, &ratio_target, &inv_sz_target);
    const float32_t *p_win        = pStatic->p_window;

    for (int32_t i = 0; i < (int32_t)N; i++)
    {
        const int16_t   pw_i16 = (int16_t)reg_ch0_i8[i] + (int16_t)reg_ch2_i8[i];
        const int16_t   ph_i16 = (int16_t)reg_ch1_i8[i] + (int16_t)reg_ch3_i8[i];
        const float32_t pw = (float32_t)pw_i16 * reg_scale - reg_2zp_scale;
        const float32_t ph = (float32_t)ph_i16 * reg_scale - reg_2zp_scale;
        const float32_t pen = _vot_penalty(pw, ph, ratio_target, inv_sz_target, penalty_k);
        pscore[i] = pen * cls_float[i] * one_mwi + p_win[i] * wi;
    }

    /* argmax(pscore) */
    uint32_t  best_idx_u;
    float32_t best_val_dummy;
    vision_models_maxi_if32ou32(pscore, (uint32_t)N, &best_val_dummy, &best_idx_u);
    const int32_t best_idx = (int32_t)best_idx_u;

    /* PSR: peak-to-sidelobe ratio on sigmoid cls map, 3x3 peak excluded */
    const float32_t psr = _vot_psr(cls_float, sz, N);

    /* Decode box at best cell */
    const uint16_t  *p_grid_x   = pStatic->p_grid_x;
    const uint16_t  *p_grid_y   = pStatic->p_grid_y;
    const float32_t  reg_zp_f   = (float32_t)pStatic->reg_zero_point;

    const float32_t r0 = ((float32_t)reg_ch0_i8[best_idx] - reg_zp_f) * reg_scale;
    const float32_t r1 = ((float32_t)reg_ch1_i8[best_idx] - reg_zp_f) * reg_scale;
    const float32_t r2 = ((float32_t)reg_ch2_i8[best_idx] - reg_zp_f) * reg_scale;
    const float32_t r3 = ((float32_t)reg_ch3_i8[best_idx] - reg_zp_f) * reg_scale;

    float32_t pred_xs, pred_ys, pred_w, pred_h;
    _vot_decode_box(p_grid_x[best_idx], p_grid_y[best_idx], r0, r1, r2, r3,
                    &pred_xs, &pred_ys, &pred_w, &pred_h);

    /* Convert search-crop space -> image coordinates, blend and assemble output */
    _vot_finalize_output(pStatic, pred_xs, pred_ys, pred_w, pred_h, pred_w, pred_h,
                          ratio_target, inv_sz_target, cls_float[best_idx], psr, pOutput);

    return AI_VOT_LIGHTTRACKM_PP_ERROR_NO;
}
