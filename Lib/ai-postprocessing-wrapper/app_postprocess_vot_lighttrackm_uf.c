/**
 ******************************************************************************
 * @file    app_postprocess_vot_lighttrackm_uf.c
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

#include "app_postprocess.h"

#if POSTPROCESS_TYPE == POSTPROCESS_VOT_LIGHTTRACKM_UF
#include <assert.h>
#include <math.h>
#include "sort.h"

/* The below structural parameters must be defined in postprocessing_config.h:
  - AI_VOT_LIGHTTRACKM_PP_SEARCH_SIZE */

#ifndef AI_VOT_LIGHTTRACKM_PP_STRIDE
/* Spatial stride of the backbone CNN. This is a model architecture constant — do not
 * change unless the backbone is modified. */
#define AI_VOT_LIGHTTRACKM_PP_STRIDE (16)
#endif

/* SCORE_SIZE is derived from SEARCH_SIZE and STRIDE */
#define AI_VOT_LIGHTTRACKM_PP_SCORE_SIZE (AI_VOT_LIGHTTRACKM_PP_SEARCH_SIZE / AI_VOT_LIGHTTRACKM_PP_STRIDE)

/* Algorithm hyperparameters (LightTrack-M tuning) - have reasonable defaults */

#ifndef AI_VOT_LIGHTTRACKM_PP_PENALTY_K
/* Size/ratio penalty coefficient: penalizes predicted boxes whose size or aspect ratio
 * deviates too much from the previous frame. Higher = stronger resistance to sudden
 * size changes. Tuned on LightTrack-M benchmarks; 0.04 is a good default. */
#define AI_VOT_LIGHTTRACKM_PP_PENALTY_K (0.04f)
#endif

#ifndef AI_VOT_LIGHTTRACKM_PP_WINDOW_INFLUENCE
/* Weight of the cosine (Hanning) spatial window blended into the score map.
 * Discourages the tracker from jumping to detections far from the previous position.
 * Higher = stronger center prior. 0.38 is a good default. */
#define AI_VOT_LIGHTTRACKM_PP_WINDOW_INFLUENCE (0.38f)
#endif

#ifndef AI_VOT_LIGHTTRACKM_PP_LR
/* Size smoothing learning rate: interpolates the new predicted size with the previous
 * one each frame (exponential moving average). Higher = faster size adaptation.
 * 0.82 is a good default (fast but not jittery). */
#define AI_VOT_LIGHTTRACKM_PP_LR (0.82f)
#endif

#ifndef AI_VOT_LIGHTTRACKM_PP_INIT_TARGET_POS_X
#define AI_VOT_LIGHTTRACKM_PP_INIT_TARGET_POS_X (0.0f)
#endif

#ifndef AI_VOT_LIGHTTRACKM_PP_INIT_TARGET_POS_Y
#define AI_VOT_LIGHTTRACKM_PP_INIT_TARGET_POS_Y (0.0f)
#endif

#ifndef AI_VOT_LIGHTTRACKM_PP_INIT_TARGET_SZ_SCALED_W
#define AI_VOT_LIGHTTRACKM_PP_INIT_TARGET_SZ_SCALED_W (1.0f)
#endif

#ifndef AI_VOT_LIGHTTRACKM_PP_INIT_TARGET_SZ_SCALED_H
#define AI_VOT_LIGHTTRACKM_PP_INIT_TARGET_SZ_SCALED_H (1.0f)
#endif

#ifndef AI_VOT_LIGHTTRACKM_PP_INIT_SCALE_Z
#define AI_VOT_LIGHTTRACKM_PP_INIT_SCALE_Z (1.0f)
#endif

#define MODEL_OUTPUT_NB (2)
#define LIGHTTRACKM_SCORE_NUMEL (AI_VOT_LIGHTTRACKM_PP_SCORE_SIZE * AI_VOT_LIGHTTRACKM_PP_SCORE_SIZE)

/* Pre-computed position grids: grid_x[i] and grid_y[i] give the x/y pixel coordinate
 * (in search-crop space) of the i-th score map cell. Used in the regression decode step
 * to convert predicted offsets (dx1, dy1, dx2, dy2) into absolute box coordinates.
 * Built once at init; values are in [0, SEARCH_SIZE-1]. */
POSTPROCESS_WRAPPER_SECTION
static uint16_t grid_x[LIGHTTRACKM_SCORE_NUMEL];
POSTPROCESS_WRAPPER_SECTION
static uint16_t grid_y[LIGHTTRACKM_SCORE_NUMEL];
/* 2-D Hanning (cosine) window of shape [SCORE_SIZE × SCORE_SIZE].
 * Multiplied into the score map before argmax to apply a spatial center prior:
 * cells near the crop center receive a higher weight, penalizing detections far
 * from the previous tracked position. Blend strength is controlled by WINDOW_INFLUENCE. */
POSTPROCESS_WRAPPER_SECTION
static float32_t hanning_window[LIGHTTRACKM_SCORE_NUMEL];
POSTPROCESS_WRAPPER_SECTION
static float32_t scratch[VOT_LIGHTTRACKM_PP_SCRATCH_SIZE(AI_VOT_LIGHTTRACKM_PP_SCORE_SIZE)];

/* will contain output index ordered by ascending output size */
static size_t output_order_index[MODEL_OUTPUT_NB];

static void build_grid_and_window(void)
{
  const int32_t score_size = AI_VOT_LIGHTTRACKM_PP_SCORE_SIZE;
  const int32_t stride = AI_VOT_LIGHTTRACKM_PP_STRIDE;
  const int32_t search_half = AI_VOT_LIGHTTRACKM_PP_SEARCH_SIZE / 2;
  float32_t hann_1d[AI_VOT_LIGHTTRACKM_PP_SCORE_SIZE];
  int32_t i;
  int32_t r;
  int32_t c;

  if (score_size == 1) {
    hann_1d[0] = 1.0f;
  } else {
    for (i = 0; i < score_size; i++) {
      const float32_t phase = (2.0f * 3.14159265358979323846f * (float32_t)i) / (float32_t)(score_size - 1);
      hann_1d[i] = 0.5f - 0.5f * cosf(phase);
    }
  }

  for (r = 0; r < score_size; r++) {
    const int32_t gy = (r - score_size / 2) * stride + search_half;
    for (c = 0; c < score_size; c++) {
      const int32_t gx = (c - score_size / 2) * stride + search_half;
      const int32_t idx = r * score_size + c;
      grid_x[idx] = (uint16_t)gx;
      grid_y[idx] = (uint16_t)gy;
      hanning_window[idx] = hann_1d[r] * hann_1d[c];
    }
  }
}

int32_t app_postprocess_init(void *params_postprocess, stai_network_info *NN_Info)
{
  vot_lighttrackm_pp_static_param_t *params = (vot_lighttrackm_pp_static_param_t *)params_postprocess;

  assert(params);
  assert(NN_Info);
  sort_model_outputs(output_order_index, MODEL_OUTPUT_NB, NN_Info);

  build_grid_and_window();

  params->target_pos_x = AI_VOT_LIGHTTRACKM_PP_INIT_TARGET_POS_X;
  params->target_pos_y = AI_VOT_LIGHTTRACKM_PP_INIT_TARGET_POS_Y;
  params->target_sz_scaled_w = AI_VOT_LIGHTTRACKM_PP_INIT_TARGET_SZ_SCALED_W;
  params->target_sz_scaled_h = AI_VOT_LIGHTTRACKM_PP_INIT_TARGET_SZ_SCALED_H;
  params->scale_z = AI_VOT_LIGHTTRACKM_PP_INIT_SCALE_Z;

  params->score_size = AI_VOT_LIGHTTRACKM_PP_SCORE_SIZE;
  params->stride = AI_VOT_LIGHTTRACKM_PP_STRIDE;
  params->search_size = AI_VOT_LIGHTTRACKM_PP_SEARCH_SIZE;

  params->penalty_k = AI_VOT_LIGHTTRACKM_PP_PENALTY_K;
  params->window_influence = AI_VOT_LIGHTTRACKM_PP_WINDOW_INFLUENCE;
  params->lr = AI_VOT_LIGHTTRACKM_PP_LR;

  params->p_grid_x = grid_x;
  params->p_grid_y = grid_y;
  params->p_window = hanning_window;
  params->p_scratch = scratch;

  return vot_lighttrackm_pp_reset(params);
}

int32_t app_postprocess_run(void *pInput[], int nb_input, void *pOutput, void *pInput_param)
{
  vot_lighttrackm_pp_in_t pp_input;

  assert(nb_input == MODEL_OUTPUT_NB);
  assert(pOutput);
  assert(pInput_param);

  pp_input.cls_raw = pInput[output_order_index[0]];
  pp_input.reg_raw = pInput[output_order_index[1]];

  return vot_lighttrackm_pp_process(&pp_input,
                                    (vot_pp_out_t *)pOutput,
                                    (const vot_lighttrackm_pp_static_param_t *)pInput_param);
}
#endif
