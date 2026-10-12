 /**
 ******************************************************************************
 * @file    app.c
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

#include "app.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_cam.h"
#include "app_config.h"
#include "nema_core.h"
#include "nema_error.h"
#include "postprocessing_config.h"
#include "app_postprocess.h"
#include "cmw_camera.h"
#include "stm32_lcd.h"
#include "stm32_lcd_ex.h"
#include "stm32n6570_discovery_lcd.h"
#include "stm32n6570_discovery_ts.h"
#include "stm32n6xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stai.h"
#include "stai_vot_template.h"
#include "stai_vot_split2.h"
#include "utils.h"

/* Verify that postprocessing config constants match the actual model dimensions */
#if AI_VOT_LIGHTTRACKM_PP_SEARCH_SIZE != STAI_VOT_SPLIT2_IN_2_WIDTH
#error "AI_VOT_LIGHTTRACKM_PP_SEARCH_SIZE must equal STAI_VOT_SPLIT2_IN_2_WIDTH"
#endif

#ifndef APP_VERSION_STRING
#define APP_VERSION_STRING "dev"
#endif

/* NN camera buffers must be RGB888, 3 bytes per pixel */
_Static_assert(NN_PIPE_FORMAT == DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1, "NN camera pipe must be RGB888");
_Static_assert(NN_PIPE_BPP == 3, "NN camera pipe must be 3 bytes per pixel");

#define FREERTOS_PRIORITY(p) ((UBaseType_t)((int)tskIDLE_PRIORITY + configMAX_PRIORITIES / 2 + (p)))

#define CACHE_OP(__op__) do { \
  if (is_cache_enable()) { \
    __op__; \
  } \
} while (0)

#define LCD_FONT Font20
#define LCD_SMALL_FONT Font16
#define UTIL_LCD_COLOR_TRANSPARENT 0
#define TEXT_COLOR UTIL_LCD_COLOR_WHITE

#define LCD_FG_WIDTH LCD_BG_WIDTH
#define LCD_FG_HEIGHT LCD_BG_HEIGHT

#define BQUEUE_MAX_BUFFERS 2
#define CPU_LOAD_HISTORY_DEPTH 4
#define METRICS_AVG_PERIOD_MS 1000
#define MAX_SIZE_CHANGE_RATIO 4.0f
#define MIN_VISIBLE_BOX_RATIO 0.80f
#define RECOVERY_MIN_VISIBLE_BOX_RATIO 0.99f
#define LAST_CROSS_SIZE 8

#define DISPLAY_BUFFER_NB (DISPLAY_DELAY + 2)

/* SiamFC context padding factor used by application tracking logic */
#define SIAMFC_CONTEXT_AMOUNT (0.5f)

/* Constant padding color used when a crop goes out of image bounds (RGB888) */
#define IMAGE_PAD_GRAY 127

/* Size of the circular NemaGFX command list used for crop and resize */
#define NEMA_CL_SIZE 8192

#define DBG_INFO 0

typedef struct
{
  uint32_t x0;
  uint32_t y0;
  uint32_t width;
  uint32_t height;
} Rectangle_TypeDef;

typedef struct
{
  float32_t x_center;
  float32_t y_center;
  float32_t width;
  float32_t height;
} Rectangle_center_TypeDef;

typedef struct {
  SemaphoreHandle_t free;
  StaticSemaphore_t free_buffer;
  SemaphoreHandle_t ready;
  StaticSemaphore_t ready_buffer;
  int32_t buffer_nb;
  uint8_t *buffers[BQUEUE_MAX_BUFFERS];
  int32_t free_idx;
  int32_t ready_idx;
} bqueue_t;

typedef struct {
  struct {
    uint64_t total;
    uint64_t thread;
    uint32_t tick;
  } history[CPU_LOAD_HISTORY_DEPTH];
} cpuload_info_t;

typedef struct {
  uint32_t nn_period_ms;
  uint32_t inf_ms;
  uint32_t preproc_ms;
  uint32_t disp_ms;
  float detection_psr;
  bool tracking_active;
  bool lost_status;
  bool display_roi_valid;
  bool preview_valid;
  Rectangle_center_TypeDef display_roi;   /* for display with smoothing */
  Rectangle_center_TypeDef last_good_roi; /* last confirmed high-confidence position */
  Rectangle_TypeDef preview_roi_lcd;      /* user selection preview in LCD coordinates */
} display_info_t;

typedef struct {
  SemaphoreHandle_t update;
  StaticSemaphore_t update_buffer;
  SemaphoreHandle_t lock;
  StaticSemaphore_t lock_buffer;
  display_info_t info;
} display_t;

typedef struct {
  Rectangle_TypeDef preview_roi_lcd; /* user selection preview in LCD coordinates */
  bool preview_valid;
  uint32_t drag_start_x;
  uint32_t drag_start_y;
  uint32_t drag_curr_x;
  uint32_t drag_curr_y;
  bool is_dragging;
} touch_state_t;

typedef struct {
  Rectangle_center_TypeDef target_roi;    /* as reported by the network */
  Rectangle_center_TypeDef last_good_roi; /* last confirmed high-confidence position */
  Rectangle_center_TypeDef display_roi;   /* for display with smoothing */
  uint32_t lost_frame_count;
  bool display_roi_valid;
  bool split2_reinit_pending;
  bool tracking_active;
  bool lost_status;
  float detection_psr;
} tracker_state_t;

/* Persistent state for the split2 network (init once, reused every frame) */
typedef struct {
  stai_ptr nn_in[STAI_VOT_SPLIT2_IN_NUM];
  stai_ptr nn_out[STAI_VOT_SPLIT2_OUT_NUM];
  vot_lighttrackm_pp_static_param_t pp_params;
  vot_pp_out_t pp_output;
  bool is_initialized;
} split2_ctx_t;

typedef struct {
  float psr;
  float fps;
  float inf_ms;
  float preproc_ms;
} metrics_t;

typedef struct {
  uint32_t sample_count;
  uint32_t window_start_ms;
  metrics_t avg;
  metrics_t acc;
} metrics_avg1s_t;

/* Globals */

/* Lcd Background Buffer : camera frame displayed on LCD */
static Rectangle_TypeDef lcd_bg_area = {
  .x0 = 0,
  .y0 = 0,
  .width = LCD_BG_WIDTH,
  .height = LCD_BG_HEIGHT,
};
static uint8_t lcd_bg_buffer[DISPLAY_BUFFER_NB][LCD_BG_WIDTH * LCD_BG_HEIGHT * 2] ALIGN_32 IN_PSRAM;
static int32_t lcd_bg_buffer_disp_idx = 1;
static int32_t lcd_bg_buffer_capt_idx = 0;

/* Lcd Foreground Buffer : target ROI, user info */
static Rectangle_TypeDef lcd_fg_area = {
  .x0 = 0,
  .y0 = 0,
  .width = LCD_FG_WIDTH,
  .height = LCD_FG_HEIGHT,
};
static uint8_t lcd_fg_buffer[2][LCD_FG_WIDTH * LCD_FG_HEIGHT* 2] ALIGN_32 IN_PSRAM;
static int32_t lcd_fg_buffer_rd_idx;

static display_t disp;
static cpuload_info_t cpu_load = {0};

/* GPU2D crop and resize context */
static GFXMMU_HandleTypeDef hgfxmmu;
static nema_cmdlist_t nema_cl;

/* NN input buffers (camera buffers for the neural network) */
static uint8_t nn_input_buffers[2][NN_PIPE_WIDTH * NN_PIPE_HEIGHT * NN_PIPE_BPP] ALIGN_32 IN_PSRAM;
static bqueue_t nn_input_queue;

/* Model */
static uint8_t split2_network_ctx[STAI_VOT_SPLIT2_CONTEXT_SIZE] ALIGN_32;
static uint8_t template_network_ctx[STAI_VOT_TEMPLATE_CONTEXT_SIZE] ALIGN_32;
static uint8_t template_embedding[STAI_VOT_TEMPLATE_OUT_1_SIZE_BYTES] ALIGN_32;

/* RTOS */
static StaticTask_t nn_thread;
static StackType_t nn_thread_stack[2 * configMINIMAL_STACK_SIZE];
static StaticTask_t dp_thread;
static StackType_t dp_thread_stack[2 *configMINIMAL_STACK_SIZE];
static StaticTask_t isp_thread;
static StackType_t isp_thread_stack[2 *configMINIMAL_STACK_SIZE];
static SemaphoreHandle_t isp_sem;
static StaticSemaphore_t isp_sem_buffer;

/* Touchscreen */
static volatile bool touch_irq_pending;
static bool touch_is_down;

static int32_t is_cache_enable(void)
{
#if defined(USE_DCACHE)
  return 1;
#else
  return 0;
#endif
}

static void cpuload_update(cpuload_info_t *cpu_load)
{
  int32_t i;

  cpu_load->history[1] = cpu_load->history[0];
  cpu_load->history[0].total = portGET_RUN_TIME_COUNTER_VALUE();
  cpu_load->history[0].thread = cpu_load->history[0].total - ulTaskGetIdleRunTimeCounter();
  cpu_load->history[0].tick = HAL_GetTick();

  if (cpu_load->history[1].tick - cpu_load->history[2].tick < 1000)
  {
    return;
  }

  for (i = 0; i < CPU_LOAD_HISTORY_DEPTH - 2; i++)
  {
    cpu_load->history[CPU_LOAD_HISTORY_DEPTH - 1 - i] = cpu_load->history[CPU_LOAD_HISTORY_DEPTH - 1 - i - 1];
  }
}

static float cpuload_get_info(const cpuload_info_t *cpu_load)
{
  uint64_t thread_delta = cpu_load->history[2].thread - cpu_load->history[3].thread;
  uint64_t total_delta = cpu_load->history[2].total - cpu_load->history[3].total;

  if (total_delta == 0)
  {
    /* No run-time elapsed between the two samples (history not filled yet after boot) */
    return 0.0f;
  }

  return 100.0f * (float)thread_delta / (float)total_delta;
}

static int32_t bqueue_init(bqueue_t *bq, int32_t buffer_nb, uint8_t **buffers)
{
  int32_t i;

  if (buffer_nb > BQUEUE_MAX_BUFFERS)
  {
    return -1;
  }

  bq->free = xSemaphoreCreateCountingStatic(buffer_nb, buffer_nb, &bq->free_buffer);
  if (!bq->free)
  {
    goto free_sem_error;
  }
  bq->ready = xSemaphoreCreateCountingStatic(buffer_nb, 0, &bq->ready_buffer);
  if (!bq->ready)
  {
    goto ready_sem_error;
  }

  bq->buffer_nb = buffer_nb;
  for (i = 0; i < buffer_nb; i++)
  {
    assert(buffers[i]);
    bq->buffers[i] = buffers[i];
  }
  bq->free_idx = 0;
  bq->ready_idx = 0;

  return 0;

ready_sem_error:
  vSemaphoreDelete(bq->free);
free_sem_error:
  return -1;
}

static uint8_t *bqueue_get_free(bqueue_t *bq, int32_t is_blocking)
{
  int32_t ret;
  int32_t is_inside_isr = xPortIsInsideInterrupt();
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint8_t *res;

  /* Blocking is not allowed from an interrupt context */
  assert(!is_inside_isr || !is_blocking);

  if (is_inside_isr)
  {
    ret = xSemaphoreTakeFromISR(bq->free, &xHigherPriorityTaskWoken);
  }
  else
  {
    ret = xSemaphoreTake(bq->free, is_blocking ? portMAX_DELAY : 0);
  }
  if (ret == pdFALSE)
  {
    return NULL;
  }

  res = bq->buffers[bq->free_idx];
  bq->free_idx = (bq->free_idx + 1) % bq->buffer_nb;

  if (is_inside_isr)
  {
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  return res;
}

static void bqueue_put_free(bqueue_t *bq)
{
  int32_t ret;

  ret = xSemaphoreGive(bq->free);
  assert(ret == pdTRUE);
}

static uint8_t *bqueue_get_ready(bqueue_t *bq)
{
  uint8_t *res;
  int32_t ret;

  ret = xSemaphoreTake(bq->ready, portMAX_DELAY);
  assert(ret == pdTRUE);

  res = bq->buffers[bq->ready_idx];
  bq->ready_idx = (bq->ready_idx + 1) % bq->buffer_nb;

  return res;
}

static void bqueue_put_ready(bqueue_t *bq)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int32_t ret;

  if (xPortIsInsideInterrupt())
  {
    ret = xSemaphoreGiveFromISR(bq->ready, &xHigherPriorityTaskWoken);
    assert(ret == pdTRUE);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  else
  {
    ret = xSemaphoreGive(bq->ready);
    assert(ret == pdTRUE);
  }
}

static void app_main_pipe_frame_event(void)
{
  int32_t next_disp_idx = (lcd_bg_buffer_disp_idx + 1) % DISPLAY_BUFFER_NB;
  int32_t next_capt_idx = (lcd_bg_buffer_capt_idx + 1) % DISPLAY_BUFFER_NB;
  HAL_StatusTypeDef ret;

  ret = HAL_DCMIPP_PIPE_SetMemoryAddress(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE1,
                                         DCMIPP_MEMORY_ADDRESS_0, (uint32_t)lcd_bg_buffer[next_capt_idx]);
  assert(ret == HAL_OK);

  ret = HAL_LTDC_SetAddress_NoReload(&hlcd_ltdc, (uint32_t)lcd_bg_buffer[next_disp_idx], LTDC_LAYER_1);
  assert(ret == HAL_OK);
  ret = HAL_LTDC_ReloadLayer(&hlcd_ltdc, LTDC_RELOAD_VERTICAL_BLANKING, LTDC_LAYER_1);
  assert(ret == HAL_OK);

  lcd_bg_buffer_disp_idx = next_disp_idx;
  lcd_bg_buffer_capt_idx = next_capt_idx;
}

static void app_ancillary_pipe_frame_event(void)
{
  uint8_t *next_buffer;
  int32_t ret;

  next_buffer = bqueue_get_free(&nn_input_queue, 0);
  if (next_buffer)
  {
    ret = HAL_DCMIPP_PIPE_SetMemoryAddress(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE2,
                                           DCMIPP_MEMORY_ADDRESS_0, (uint32_t)next_buffer);
    assert(ret == HAL_OK);
    bqueue_put_ready(&nn_input_queue);
  }
}

static void app_main_pipe_vsync_event(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int32_t ret;

  ret = xSemaphoreGiveFromISR(isp_sem, &xHigherPriorityTaskWoken);
  if (ret == pdTRUE)
  {
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/* Compute the side length of the context-padded square crop (SiamFC convention).
 * A square of side s_z centered on the target encompasses the target plus a surrounding
 * context margin, giving the network enough scene context to build a robust embedding:
 *   context = CONTEXT_AMOUNT * (w + h)
 *   s_z     = sqrt((w + context) * (h + context)) */
static float app_siamfc_ctx_sz(float w, float h)
{
  float context = SIAMFC_CONTEXT_AMOUNT * (w + h);

  return sqrtf((w + context) * (h + context));
}

/* Update pp_params tracking state from object position and size in image coordinates */
static void app_update_pp_geometry(vot_lighttrackm_pp_static_param_t *p, const Rectangle_center_TypeDef *roi)
{
  float s_z = app_siamfc_ctx_sz(roi->width, roi->height);

  p->target_pos_x       = roi->x_center;
  p->target_pos_y       = roi->y_center;
  p->scale_z            = (float)STAI_VOT_TEMPLATE_IN_1_WIDTH / s_z;
  p->target_sz_scaled_w = roi->width * p->scale_z;
  p->target_sz_scaled_h = roi->height * p->scale_z;
}

/* Bind the rgb888 destination buffer to the GPU.
 * The GPU2D cannot write 24bpp pixels, so the buffer is reached through a GFXMMU virtual
 * buffer declared as RGBA8888 whose packing drops the most significant byte of every
 * word: what actually lands in physical memory is rgb888 */
static void app_gpu_bind_dst_rgb888(uint8_t *p_dst, uint32_t dst_w, uint32_t dst_h)
{
  GFXMMU_BuffersTypeDef buffers = {0};
  int32_t ret;

  buffers.Buf0Address = (uint32_t)p_dst;
  ret = HAL_GFXMMU_ModifyBuffers(&hgfxmmu, &buffers);
  assert(ret == HAL_OK);

  /* Drop any dirty cache line still covering the destination so that it cannot be
   * written back on top of what the GPU is about to produce */
  CACHE_OP(SCB_CleanInvalidateDCache_by_Addr(p_dst, dst_w * dst_h * NN_PIPE_BPP));

  nema_bind_dst_tex(GFXMMU_VIRTUAL_BUFFER0_BASE, dst_w, dst_h, NEMA_RGBA8888, -1);
  nema_set_clip(0, 0, dst_w, dst_h);
}

/* Run the pending command list and make the result visible to the CPU and to the NPU */
static void app_gpu_run(uint8_t *p_dst, uint32_t dst_w, uint32_t dst_h)
{
  nema_cl_submit(&nema_cl);
  nema_cl_wait(&nema_cl);
  assert(!nema_get_error());

  CACHE_OP(SCB_InvalidateDCache_by_Addr(p_dst, dst_w * dst_h * NN_PIPE_BPP));
}

/* Fill the whole destination with the constant padding color */
static void app_gpu_fill_rgb888(uint8_t *p_dst, uint32_t dst_w, uint32_t dst_h)
{
  app_gpu_bind_dst_rgb888(p_dst, dst_w, dst_h);
  nema_clear(nema_rgba(IMAGE_PAD_GRAY, IMAGE_PAD_GRAY, IMAGE_PAD_GRAY, 0xff));
  app_gpu_run(p_dst, dst_w, dst_h);
}

/* Bilinear crop and resize of a source sub-rectangle into a destination sub-rectangle.
 * When fill_padding is set, the destination is first cleared with the padding color */
static void app_gpu_resize_rgb888(uint8_t *p_src, uint32_t src_w, uint32_t src_h,
                                  uint8_t *p_dst, uint32_t dst_w, uint32_t dst_h,
                                  uint32_t dst_x, uint32_t dst_y, uint32_t dst_sub_w, uint32_t dst_sub_h,
                                  uint32_t src_x, uint32_t src_y, uint32_t src_sub_w, uint32_t src_sub_h,
                                  bool fill_padding)
{
  app_gpu_bind_dst_rgb888(p_dst, dst_w, dst_h);

  if (fill_padding)
  {
    nema_clear(nema_rgba(IMAGE_PAD_GRAY, IMAGE_PAD_GRAY, IMAGE_PAD_GRAY, 0xff));
  }

  nema_bind_src_tex((uintptr_t)p_src, src_w, src_h, NEMA_RGB24, src_w * NN_PIPE_BPP, NEMA_FILTER_BL);
  nema_set_blend_blit(NEMA_BL_SRC);
  nema_blit_subrect_fit((int32_t)dst_x, (int32_t)dst_y, (int32_t)dst_sub_w, (int32_t)dst_sub_h,
                        (int32_t)src_x, (int32_t)src_y, (int32_t)src_sub_w, (int32_t)src_sub_h);

  app_gpu_run(p_dst, dst_w, dst_h);
}

static void app_gpu_init(void)
{
  GFXMMU_PackingTypeDef packing;
  int32_t ret;

  memset(&packing, 0, sizeof(packing));

  ret = nema_init();
  assert(ret == 0);
  assert(!nema_get_error());

  hgfxmmu.Instance = GFXMMU;
  hgfxmmu.Init.BlockSize = GFXMMU_12BYTE_BLOCKS;
  hgfxmmu.Init.AddressTranslation = DISABLE;
  ret = HAL_GFXMMU_Init(&hgfxmmu);
  assert(ret == HAL_OK);

  packing.Buffer0Activation = ENABLE;
  packing.Buffer0Mode = GFXMMU_PACKING_MSB_REMOVE;
  packing.DefaultAlpha = 0xff;
  ret = HAL_GFXMMU_ConfigPacking(&hgfxmmu, &packing);
  assert(ret == HAL_OK);

  nema_cl = nema_cl_create_sized(NEMA_CL_SIZE);
  assert(nema_cl.bo.base_virt);
  nema_cl_bind_circular(&nema_cl);
}

/* Image crop and resize (rgb888): gray-filled output.
 * When crop goes out of bounds, only the in-image part is resized into its destination */
static void app_crop_resize_rgb888(uint8_t *p_src, uint32_t src_w, uint32_t src_h,
                                   int32_t crop_x, int32_t crop_y, uint32_t crop_w, uint32_t crop_h,
                                   uint8_t *p_dst, uint32_t dst_w, uint32_t dst_h)
{
  uint32_t src_iw, src_ih, dx0, dy0, dx1, dy1, dst_sub_w, dst_sub_h;
  int32_t ix0, iy0, ix1, iy1;
  bool needs_padding;

  assert(p_src);
  assert(p_dst);
  assert(crop_w > 0);
  assert(crop_h > 0);
  assert(dst_w > 0);
  assert(dst_h > 0);

  needs_padding = (crop_x < 0) || (crop_y < 0) ||
                  ((uint32_t)crop_x + crop_w > src_w) || ((uint32_t)crop_y + crop_h > src_h);

  /* Fast path: no padding needed, resize the whole crop directly */
  if (!needs_padding)
  {
    app_gpu_resize_rgb888(p_src, src_w, src_h, p_dst, dst_w, dst_h,
                          0, 0, dst_w, dst_h, crop_x, crop_y, crop_w, crop_h, false);
    return;
  }

  /* Intersection of crop rectangle with source image */
  ix0 = MAX(0, crop_x);
  iy0 = MAX(0, crop_y);
  ix1 = MIN((int32_t)src_w, crop_x + (int32_t)crop_w);
  iy1 = MIN((int32_t)src_h, crop_y + (int32_t)crop_h);

  if (ix1 <= ix0 || iy1 <= iy0)
  {
    app_gpu_fill_rgb888(p_dst, dst_w, dst_h);
    return;
  }

  src_iw = (uint32_t)(ix1 - ix0);
  src_ih = (uint32_t)(iy1 - iy0);

  /* Map intersection to destination sub-rectangle using the crop scale */
  dx0 = (uint32_t)(((uint64_t)(ix0 - crop_x) * dst_w) / crop_w);
  dy0 = (uint32_t)(((uint64_t)(iy0 - crop_y) * dst_h) / crop_h);
  dx1 = (uint32_t)((((uint64_t)(ix1 - crop_x) * dst_w) + crop_w - 1) / crop_w);
  dy1 = (uint32_t)((((uint64_t)(iy1 - crop_y) * dst_h) + crop_h - 1) / crop_h);

  dx0 = MIN(dx0, dst_w);
  dy0 = MIN(dy0, dst_h);
  dx1 = MIN(dx1, dst_w);
  dy1 = MIN(dy1, dst_h);

  if (dx1 <= dx0 || dy1 <= dy0)
  {
    app_gpu_fill_rgb888(p_dst, dst_w, dst_h);
    return;
  }

  dst_sub_w = dx1 - dx0;
  dst_sub_h = dy1 - dy0;

  /* Resize the in-image intersection into the mapped destination sub-rectangle,
   * the rest of the destination is filled with the constant padding color */
  app_gpu_resize_rgb888(p_src, src_w, src_h, p_dst, dst_w, dst_h,
                        dx0, dy0, dst_sub_w, dst_sub_h, ix0, iy0, src_iw, src_ih, true);
}

static void app_touch_set_preview(touch_state_t *touch, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1)
{
  uint32_t lx0, ly0, lx1, ly1;

  lx0 = MIN(x0, x1);
  ly0 = MIN(y0, y1);
  lx1 = MAX(x0, x1);
  ly1 = MAX(y0, y1);

  touch->preview_roi_lcd.x0 = lx0;
  touch->preview_roi_lcd.y0 = ly0;
  touch->preview_roi_lcd.width = lx1 - lx0 + 1;
  touch->preview_roi_lcd.height = ly1 - ly0 + 1;
  touch->preview_valid = true;
}

static void app_touch_clear_preview(touch_state_t *touch)
{
  touch->preview_roi_lcd = (Rectangle_TypeDef){0};
  touch->preview_valid = false;
}

static uint8_t app_touch_build_selection(const touch_state_t *touch, Rectangle_TypeDef *out_nn_roi)
{
  uint32_t lx0, ly0, lx1, ly1;

  if (touch->drag_start_x == touch->drag_curr_x || touch->drag_start_y == touch->drag_curr_y)
  {
    /* empty selection */
    return 0;
  }

  lx0 = MIN(touch->drag_start_x, touch->drag_curr_x);
  ly0 = MIN(touch->drag_start_y, touch->drag_curr_y);
  lx1 = MAX(touch->drag_start_x, touch->drag_curr_x);
  ly1 = MAX(touch->drag_start_y, touch->drag_curr_y);

  out_nn_roi->x0 =  (lx0 * NN_PIPE_WIDTH) / LCD_FG_WIDTH;
  out_nn_roi->y0 =  (ly0 * NN_PIPE_HEIGHT) / LCD_FG_HEIGHT;
  out_nn_roi->width = ((lx1 * NN_PIPE_WIDTH) / LCD_FG_WIDTH) - out_nn_roi->x0 + 1;
  out_nn_roi->height = ((ly1 * NN_PIPE_HEIGHT) / LCD_FG_HEIGHT) - out_nn_roi->y0 + 1;
  return 1;
}

static void app_touch_start_selection_from_point(touch_state_t *touch, uint32_t x, uint32_t y)
{
  touch->drag_start_x = MIN(x, LCD_FG_WIDTH - 1);
  touch->drag_start_y = MIN(y, LCD_FG_HEIGHT - 1);
  touch->drag_curr_x = touch->drag_start_x;
  touch->drag_curr_y = touch->drag_start_y;
  touch->is_dragging = true;
  app_touch_set_preview(touch, touch->drag_start_x, touch->drag_start_y, touch->drag_curr_x, touch->drag_curr_y);
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == TS_INT_PIN)
  {
    touch_irq_pending = true;
  }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == TS_INT_PIN)
  {
    touch_irq_pending = true;
  }
}

static void app_touch_enable_it(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  TS_INT_GPIO_CLK_ENABLE();

  gpio_init.Pin = TS_INT_PIN;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Mode = GPIO_MODE_IT_RISING_FALLING;
  HAL_GPIO_Init(TS_INT_GPIO_PORT, &gpio_init);

  HAL_NVIC_SetPriority(TS_INT_EXTI_IRQn, BSP_TS_IT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(TS_INT_EXTI_IRQn);
}

static bool app_touch_read(TS_State_t *ts_state)
{
  assert(ts_state != NULL);

  if (!touch_irq_pending && !touch_is_down)
  {
    /* optimised: skip I2C read if no new touch event is pending and no finger is down */
    *ts_state = (TS_State_t){0};
    return true;
  }
  touch_irq_pending = false;

  if (BSP_TS_GetState(0, ts_state) != BSP_ERROR_NONE)
  {
    *ts_state = (TS_State_t){0};
    touch_is_down = false;
    return false;
  }

  touch_is_down = (ts_state->TouchDetected != 0);

  return true;
}

static uint8_t app_touch_update_selection(touch_state_t *touch, const TS_State_t *ts_state,
                                          Rectangle_TypeDef *out_nn_roi)
{
  uint8_t selection_ready = 0;

  assert(touch != NULL);
  assert(ts_state != NULL);

  if ((ts_state->TouchDetected == 0) && (touch->is_dragging == false))
  {
    return selection_ready;
  }

  if (ts_state->TouchDetected)
  {
    /* Touchscreen pressed */
    if (touch->is_dragging == false)
    {
      /* Start selection */
      app_touch_start_selection_from_point(touch, ts_state->TouchX, ts_state->TouchY);
    }
    else
    {
      /* Selection being updated */
      touch->drag_curr_x = MIN(ts_state->TouchX, LCD_FG_WIDTH - 1);
      touch->drag_curr_y = MIN(ts_state->TouchY, LCD_FG_HEIGHT - 1);
    }

    /* Display ongoing selection (not completed yet) */
    app_touch_set_preview(touch, touch->drag_start_x, touch->drag_start_y, touch->drag_curr_x, touch->drag_curr_y);
  }
  else
  {
    /* Touchscreen NOT pressed */
    if (touch->is_dragging)
    {
      /* Touchscreen releases after user selection */
      touch->is_dragging = false;
      app_touch_clear_preview(touch);

      selection_ready = app_touch_build_selection(touch, out_nn_roi);
    }
  }

  return selection_ready;
}

static float app_box_visible_ratio(const Rectangle_center_TypeDef *roi)
{
  float x1, y1, x2, y2, vx1, vy1, vx2, vy2, box_area, visible_area;

  x1 = roi->x_center - roi->width / 2.0f;
  y1 = roi->y_center - roi->height / 2.0f;
  x2 = roi->x_center + roi->width / 2.0f;
  y2 = roi->y_center + roi->height / 2.0f;

  box_area = MAX(0.0f, x2 - x1) * MAX(0.0f, y2 - y1);
  if (box_area <= 0.0f)
  {
    return 0.0f;
  }

  vx1 = MAX(0.0f, MIN((float)NN_PIPE_WIDTH, x1));
  vy1 = MAX(0.0f, MIN((float)NN_PIPE_HEIGHT, y1));
  vx2 = MAX(0.0f, MIN((float)NN_PIPE_WIDTH, x2));
  vy2 = MAX(0.0f, MIN((float)NN_PIPE_HEIGHT, y2));
  visible_area = MAX(0.0f, vx2 - vx1) * MAX(0.0f, vy2 - vy1);

  return visible_area / box_area;
}

static bool app_compute_lost_status(const Rectangle_center_TypeDef *prev_roi,
                                    const Rectangle_center_TypeDef *new_roi, float psr)
{
  bool lost_status_psr, lost_status_size_exploded, lost_status_min_visible;
  float prev_size, new_size, size_ratio;

  /* Lost if PSR is too low OR size changed a lot OR target is out of screen */
  lost_status_psr = (psr < PSR_LOST_THRESHOLD);

  prev_size = prev_roi->width * prev_roi->height;
  new_size = new_roi->width * new_roi->height;
  size_ratio = MAX(new_size, prev_size) / (MIN(new_size, prev_size) + 1e-6f);
  lost_status_size_exploded = (size_ratio > MAX_SIZE_CHANGE_RATIO);

  lost_status_min_visible = (app_box_visible_ratio(new_roi) < MIN_VISIBLE_BOX_RATIO);

  return (lost_status_psr || lost_status_size_exploded || lost_status_min_visible);
}

/* Update the ROI used for display with a simple smoothing */
static void app_simple_smooth_display_roi(tracker_state_t *tracker, const Rectangle_center_TypeDef *raw_roi)
{
  if (tracker->display_roi_valid == false)
  {
    /* First frame after a (re)selection: adopt the raw ROI without smoothing */
    tracker->display_roi = *raw_roi;
    tracker->display_roi_valid = true;
    return;
  }

  /* Simple smoothing: average the previous display ROI with the new raw ROI */
  tracker->display_roi.x_center = 0.5f * (tracker->display_roi.x_center + raw_roi->x_center);
  tracker->display_roi.y_center = 0.5f * (tracker->display_roi.y_center + raw_roi->y_center);
  tracker->display_roi.width    = 0.5f * (tracker->display_roi.width + raw_roi->width);
  tracker->display_roi.height   = 0.5f * (tracker->display_roi.height + raw_roi->height);
}

static metrics_t app_update_metrics_avg_1s(metrics_avg1s_t *state, const metrics_t *sample)
{
  metrics_t values;
  uint32_t now_ms;

  assert(state != NULL);
  assert(sample != NULL);

  now_ms = HAL_GetTick();
  state->acc.psr += sample->psr;
  state->acc.fps += sample->fps;
  state->acc.inf_ms += sample->inf_ms;
  state->acc.preproc_ms += sample->preproc_ms;
  state->sample_count++;

  if (state->window_start_ms == 0)
  {
    /* First call, no average */
    state->window_start_ms = now_ms;
    state->avg = *sample;
  }
  else if ((now_ms - state->window_start_ms) >= METRICS_AVG_PERIOD_MS)
  {
    /* 1 second window : compute average */
    state->avg.psr = state->acc.psr / state->sample_count;
    state->avg.fps = state->acc.fps / state->sample_count;
    state->avg.inf_ms = state->acc.inf_ms / state->sample_count;
    state->avg.preproc_ms = state->acc.preproc_ms / state->sample_count;
    state->acc = (metrics_t){0};
    state->sample_count = 0;
    state->window_start_ms = now_ms;
  }

  values = state->avg;

  return values;
}

static void app_draw_rect_2px(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color)
{
  UTIL_LCD_DrawRect(x, y, w, h, color);
  if ((w > 2) && (h > 2))
  {
    UTIL_LCD_DrawRect(x + 1, y + 1, w - 2, h - 2, color);
  }
}

static void app_draw_cross_center(const Rectangle_center_TypeDef *roi)
{
  int32_t cx, cy, cx2, cy2, x0, x1, y0, y1;

  assert(roi != NULL);
  if ((roi->width <= 0.0f) || (roi->height <= 0.0f))
  {
    return;
  }

  cx = (int32_t)roundf(roi->x_center * LCD_FG_WIDTH / NN_PIPE_WIDTH);
  cy = (int32_t)roundf(roi->y_center * LCD_FG_HEIGHT / NN_PIPE_HEIGHT);
  cx = MAX(0, MIN(cx, (int32_t)LCD_FG_WIDTH - 1));
  cy = MAX(0, MIN(cy, (int32_t)LCD_FG_HEIGHT - 1));

  x0 = MAX(0, cx - LAST_CROSS_SIZE);
  x1 = MIN((int32_t)LCD_FG_WIDTH - 1, cx + LAST_CROSS_SIZE);
  y0 = MAX(0, cy - LAST_CROSS_SIZE);
  y1 = MIN((int32_t)LCD_FG_HEIGHT - 1, cy + LAST_CROSS_SIZE);
  cx2 = (cx < (int32_t)LCD_FG_WIDTH - 1) ? cx + 1 : cx - 1;
  cy2 = (cy < (int32_t)LCD_FG_HEIGHT - 1) ? cy + 1 : cy - 1;

  UTIL_LCD_DrawLine((uint32_t)x0, (uint32_t)cy, (uint32_t)x1, (uint32_t)cy, UTIL_LCD_COLOR_RED);
  UTIL_LCD_DrawLine((uint32_t)x0, (uint32_t)cy2, (uint32_t)x1, (uint32_t)cy2, UTIL_LCD_COLOR_RED);
  UTIL_LCD_DrawLine((uint32_t)cx, (uint32_t)y0, (uint32_t)cx, (uint32_t)y1, UTIL_LCD_COLOR_RED);
  UTIL_LCD_DrawLine((uint32_t)cx2, (uint32_t)y0, (uint32_t)cx2, (uint32_t)y1, UTIL_LCD_COLOR_RED);
}

static void dp_update_network_output(display_info_t *info)
{
  static metrics_avg1s_t metrics_avg_state = {0};
  uint32_t current_color, detect_color = UTIL_LCD_COLOR_GREEN;
  int32_t rx, ry, rw, rh, line_nb = 0;
  float cpu_load_one_second, nn_fps;
  metrics_t sample, avg;

  /* Clear previous ui */
  UTIL_LCD_FillRect(lcd_fg_area.x0, lcd_fg_area.y0, lcd_fg_area.width, lcd_fg_area.height, 0);
  CACHE_OP(SCB_InvalidateDCache_by_Addr(lcd_fg_buffer[lcd_fg_buffer_rd_idx], LCD_FG_WIDTH * LCD_FG_HEIGHT * 2));

  /* Cpu load */
  cpuload_update(&cpu_load);
  cpu_load_one_second = cpuload_get_info(&cpu_load);

  /* General metrics */
  nn_fps = (info->nn_period_ms > 0) ? (1000.0f / (float)info->nn_period_ms) : 0.0f;
  sample = (metrics_t){
    .psr = info->detection_psr,
    .fps = nn_fps,
    .inf_ms = (float)info->inf_ms,
    .preproc_ms = (float)info->preproc_ms,
  };
  avg = app_update_metrics_avg_1s(&metrics_avg_state, &sample);
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Inference");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%ums", (uint32_t)(avg.inf_ms));
  line_nb += 2;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Preproc");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, " %ums", (uint32_t)(avg.preproc_ms));
  line_nb += 2;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%.1f FPS", avg.fps);
  if (DBG_INFO)
  {
    line_nb += 2;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Display");
    line_nb += 1;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%ums", info->disp_ms);
    line_nb += 2;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "CPU load");
    line_nb += 1;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%.1f%%", cpu_load_one_second);
    line_nb += 1;
  }

  /* Model specific metrics */
  line_nb = 0;
  if (info->tracking_active)
  {
    /* We are during tracking detection : display average PSR, lost status and draw ROI around target */
    detect_color = (avg.psr < PSR_LOST_THRESHOLD) ? UTIL_LCD_COLOR_RED :
                   (avg.psr < PSR_WARN_THRESHOLD) ? UTIL_LCD_COLOR_ORANGE :
                   UTIL_LCD_COLOR_GREEN;

    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), LEFT_MODE, "Detection PSR");
    line_nb += 1;
    current_color = UTIL_LCD_GetTextColor();
    UTIL_LCD_SetTextColor(detect_color);
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), LEFT_MODE, "%3d", (int32_t)avg.psr);
    UTIL_LCD_SetTextColor(current_color);

    if (info->lost_status)
    {
      current_color = UTIL_LCD_GetTextColor();
      UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_RED);
      UTIL_LCDEx_PrintfAt((4 + 1) * (LCD_FONT.Width), LINE(line_nb), LEFT_MODE, "LOST");
      UTIL_LCD_SetFont(&LCD_SMALL_FONT);
      UTIL_LCDEx_PrintfAt(0, LCD_FG_HEIGHT - LINE(1), RIGHT_MODE, "+ last known position");
      UTIL_LCD_SetFont(&LCD_FONT);
      UTIL_LCD_SetTextColor(current_color);

      app_draw_cross_center(&info->last_good_roi);
    }
    else if (info->display_roi_valid && info->display_roi.width > 0.0f && info->display_roi.height > 0.0f)
    {
      /* Draw target ROI rectangle */
      rx = (int32_t)roundf((info->display_roi.x_center - info->display_roi.width  / 2.0f) * LCD_FG_WIDTH  / NN_PIPE_WIDTH);
      ry = (int32_t)roundf((info->display_roi.y_center - info->display_roi.height / 2.0f) * LCD_FG_HEIGHT / NN_PIPE_HEIGHT);
      rw = (int32_t)roundf(info->display_roi.width  * LCD_FG_WIDTH  / NN_PIPE_WIDTH);
      rh = (int32_t)roundf(info->display_roi.height * LCD_FG_HEIGHT / NN_PIPE_HEIGHT);

      /* Clamp to display bounds */
      rx = MAX(0, MIN(rx, (int32_t)LCD_FG_WIDTH - 1));
      ry = MAX(0, MIN(ry, (int32_t)LCD_FG_HEIGHT - 1));
      rw = MAX(1, MIN(rw, (int32_t)LCD_FG_WIDTH - rx));
      rh = MAX(1, MIN(rh, (int32_t)LCD_FG_HEIGHT - ry));
      app_draw_rect_2px((uint32_t)rx, (uint32_t)ry, (uint32_t)rw, (uint32_t)rh, detect_color);
    }
  }
  else
  {
    /* We are during user selection : display selection messages and draw user selection rectangle */
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), LEFT_MODE, "Touch and drag to select an area.");
    line_nb += 1;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), LEFT_MODE, "Release to confirm.");

    if (info->preview_valid && (info->preview_roi_lcd.width > 0) && (info->preview_roi_lcd.height > 0))
    {
      /* Draw user selection rectangle */
      rx = (int32_t)info->preview_roi_lcd.x0;
      ry = (int32_t)info->preview_roi_lcd.y0;
      rw = (int32_t)info->preview_roi_lcd.width;
      rh = (int32_t)info->preview_roi_lcd.height;

      rx = MAX(0, MIN(rx, (int32_t)LCD_FG_WIDTH - 1));
      ry = MAX(0, MIN(ry, (int32_t)LCD_FG_HEIGHT - 1));
      rw = MAX(1, MIN(rw, (int32_t)LCD_FG_WIDTH - rx));
      rh = MAX(1, MIN(rh, (int32_t)LCD_FG_HEIGHT - ry));

      app_draw_rect_2px((uint32_t)rx, (uint32_t)ry, (uint32_t)rw, (uint32_t)rh, UTIL_LCD_COLOR_BLUE);
    }
  }
}

/* Run the template network and store the resulting embedding into template_embedding */
static void app_nn_template_process(tracker_state_t *tracker, uint8_t *capture_buffer,
                                    const Rectangle_TypeDef *user_sel)
{
  uint32_t s_z, nn_w = STAI_VOT_TEMPLATE_IN_1_WIDTH, nn_h = STAI_VOT_TEMPLATE_IN_1_HEIGHT;
  int32_t tmpl_cx, tmpl_cy, tmpl_x0, tmpl_y0, ret;
  stai_size nn_in_num, nn_out_num;
  stai_ptr nn_in, nn_out;
  stai_network_info info;

  assert(user_sel != NULL);

  /* Prepare the template network */
  ret = stai_vot_template_init(template_network_ctx);
  assert(ret == STAI_SUCCESS);

  ret = stai_vot_template_get_info(template_network_ctx, &info);
  assert(ret == STAI_SUCCESS);
  assert(info.outputs[0].size_bytes == sizeof(template_embedding));

  /* Get input pointer (image template) */
  ret = stai_vot_template_get_inputs(template_network_ctx, &nn_in, &nn_in_num);
  assert(ret == STAI_SUCCESS);
  assert(nn_in_num == 1);

  /* Get output pointer (template embedding) */
  ret = stai_vot_template_get_outputs(template_network_ctx, &nn_out, &nn_out_num);
  assert(ret == STAI_SUCCESS);
  assert(nn_out_num == 1);

  /* Define a square crop area larger than the template following the SiamFC context-padding convention */
  s_z = (uint32_t)roundf(app_siamfc_ctx_sz((float)user_sel->width, (float)user_sel->height));
  tmpl_cx = (int32_t)(user_sel->x0 + user_sel->width / 2);
  tmpl_cy = (int32_t)(user_sel->y0 + user_sel->height / 2);
  tmpl_x0 = tmpl_cx - (int32_t)(s_z / 2);
  tmpl_y0 = tmpl_cy - (int32_t)(s_z / 2);

  /* Do crop and resize */
  app_crop_resize_rgb888(capture_buffer, NN_PIPE_WIDTH, NN_PIPE_HEIGHT,
                         tmpl_x0, tmpl_y0, s_z, s_z, nn_in, nn_w, nn_h);

  /* Clean invalidate buffer before Hw access it */
  CACHE_OP(SCB_CleanInvalidateDCache_by_Addr(nn_in, STAI_VOT_TEMPLATE_IN_1_SIZE_BYTES));

  /* The capture buffer is not needed anymore: release it before the inference */
  bqueue_put_free(&nn_input_queue);

  /* Run ATON inference */
  ret = stai_vot_template_run(template_network_ctx, STAI_MODE_SYNC);
  assert(ret == STAI_SUCCESS);

  /* Copy the output to template embedding */
  CACHE_OP(SCB_InvalidateDCache_by_Addr(nn_out, info.outputs[0].size_bytes));
  memcpy(template_embedding, nn_out, sizeof(template_embedding));

  /* Deinit */
  ret = stai_vot_template_deinit(template_network_ctx);
  assert(ret == STAI_SUCCESS);

  /* Initialize target with user selection */
  tracker->target_roi.height   = (float)user_sel->height;
  tracker->target_roi.width    = (float)user_sel->width;
  tracker->target_roi.x_center = (float)(user_sel->x0 + user_sel->width / 2);
  tracker->target_roi.y_center = (float)(user_sel->y0 + user_sel->height / 2);
  tracker->last_good_roi       = tracker->target_roi;
  tracker->lost_frame_count    = 0;

  /* Restart the display smoothing from this new target */
  tracker->display_roi_valid = false;
}

/* (Re)initialize the split2 network and post-processing when needed */
static void app_split2_ensure_init(split2_ctx_t *ctx, tracker_state_t *tracker)
{
  stai_size nn_in_num, nn_out_num;
  stai_network_info info;
  int32_t ret;

  if (ctx->is_initialized && tracker->split2_reinit_pending)
  {
    /* Deinitialize the split2 network if reinitialization is pending */
    ret = stai_vot_split2_deinit(split2_network_ctx);
    assert(ret == STAI_SUCCESS);
    ctx->is_initialized = false;
  }

  if (!ctx->is_initialized || tracker->split2_reinit_pending)
  {
    /* Initialise the split2 network */
    ret = stai_vot_split2_init(split2_network_ctx);
    assert(ret == STAI_SUCCESS);

    ret = stai_vot_split2_get_info(split2_network_ctx, &info);
    assert(ret == STAI_SUCCESS);
    assert(info.inputs[0].size_bytes == sizeof(template_embedding));

    /* Get input pointers (embedding + search zone image) */
    ret = stai_vot_split2_get_inputs(split2_network_ctx, ctx->nn_in, &nn_in_num);
    assert(ret == STAI_SUCCESS);
    assert(nn_in_num == 2);

    /* Get output pointers (cls + reg) */
    ret = stai_vot_split2_get_outputs(split2_network_ctx, ctx->nn_out, &nn_out_num);
    assert(ret == STAI_SUCCESS);
    assert(nn_out_num == 2);

    /* Initialize post-processing */
    ret = app_postprocess_init(&ctx->pp_params, &info);
    assert(ret == AI_VOT_LIGHTTRACKM_PP_ERROR_NO);

    ctx->is_initialized = true;
    tracker->split2_reinit_pending = false;
  }
}

/* Crop and resize the search zone into the NN input buffer */
static void app_split2_preproc(split2_ctx_t *ctx, uint8_t *capture_buffer,
                               const Rectangle_center_TypeDef *ref_roi, uint32_t *preproc_ms)
{
  uint32_t ts, s_x, nn_w = STAI_VOT_SPLIT2_IN_2_WIDTH, nn_h = STAI_VOT_SPLIT2_IN_2_HEIGHT;
  int32_t x0, y0;

  /* Derive the search crop side from the scale already stored in pp_params:
   *   s_x = nn_w / scale_z
   * (equivalent to the SiamFC s_z + 2*d_search formula, simplified).
   * The crop is centered at ref_roi and resized to SEARCH_SIZE×SEARCH_SIZE */
  ts  = HAL_GetTick();
  s_x = (uint32_t)roundf((float)nn_w / ctx->pp_params.scale_z);
  x0  = (int32_t)roundf(ref_roi->x_center) - (int32_t)(s_x / 2);
  y0  = (int32_t)roundf(ref_roi->y_center) - (int32_t)(s_x / 2);

  app_crop_resize_rgb888(capture_buffer, NN_PIPE_WIDTH, NN_PIPE_HEIGHT, x0, y0, s_x, s_x, ctx->nn_in[1], nn_w, nn_h);

  CACHE_OP(SCB_CleanInvalidateDCache_by_Addr(ctx->nn_in[1], STAI_VOT_SPLIT2_IN_2_SIZE_BYTES));
  *preproc_ms = HAL_GetTick() - ts;
}

/* Copy embedding and run ATON inference */
static void app_split2_infer(split2_ctx_t *ctx, uint32_t *inf_ms)
{
  uint32_t ts;
  int32_t ret;

  /* Fill in the embedding input (must be done for each run since NN reuses input memory internally) */
  memcpy(ctx->nn_in[0], template_embedding, sizeof(template_embedding));
  CACHE_OP(SCB_CleanInvalidateDCache_by_Addr(ctx->nn_in[0], sizeof(template_embedding)));

  /* Run ATON inference */
  ts = HAL_GetTick();
  ret = stai_vot_split2_run(split2_network_ctx, STAI_MODE_SYNC);
  assert(ret == STAI_SUCCESS);
  *inf_ms = HAL_GetTick() - ts;

  /* Make data available for SW */
  CACHE_OP(SCB_InvalidateDCache_by_Addr(ctx->nn_out[0], STAI_VOT_SPLIT2_OUT_1_SIZE_BYTES));
  CACHE_OP(SCB_InvalidateDCache_by_Addr(ctx->nn_out[1], STAI_VOT_SPLIT2_OUT_2_SIZE_BYTES));
}

/* Update tracker state from post-processing output, handling lost/recovery logic */
static void app_split2_update_tracker(tracker_state_t *tracker, const vot_pp_out_t *pp_out)
{
  Rectangle_center_TypeDef prev_roi, new_roi;
  bool was_lost;

  was_lost = tracker->lost_status;
  prev_roi = tracker->target_roi;

  new_roi.x_center = pp_out->x_center;
  new_roi.y_center = pp_out->y_center;
  new_roi.width    = pp_out->width;
  new_roi.height   = pp_out->height;

  if (was_lost)
  {
    /* Recovery requires a strong PSR and a geometrically sane box */
    if ((pp_out->psr >= PSR_WARN_THRESHOLD) &&
        (app_box_visible_ratio(&new_roi) >= RECOVERY_MIN_VISIBLE_BOX_RATIO))
    {
      tracker->lost_status      = false;
      tracker->lost_frame_count = 0;
      tracker->target_roi       = new_roi;
      tracker->last_good_roi    = new_roi;
      app_simple_smooth_display_roi(tracker, &new_roi);
    }
    else
    {
      /* Stay lost: freeze target_roi at last good position for the next frame */
      tracker->lost_status = true;
      tracker->lost_frame_count++;
      tracker->target_roi = tracker->last_good_roi;
    }
  }
  else
  {
    tracker->lost_status = app_compute_lost_status(&prev_roi, &new_roi, pp_out->psr);
    if (tracker->lost_status)
    {
      /* Newly lost: freeze target_roi at last good position */
      tracker->lost_frame_count = 1;
      tracker->target_roi = tracker->last_good_roi;
    }
    else
    {
      tracker->lost_frame_count = 0;
      tracker->target_roi = new_roi;
      /* Update last good position only when confidence is high */
      if (pp_out->psr >= PSR_WARN_THRESHOLD)
      {
        tracker->last_good_roi = new_roi;
      }
      app_simple_smooth_display_roi(tracker, &new_roi);
    }
  }
}

/* Run the split2 network on the captured frame and using template_embedding as second input */
static void app_nn_split2_process(tracker_state_t *tracker, uint8_t *capture_buffer,
                                  uint32_t *inf_ms, uint32_t *preproc_ms)
{
  static split2_ctx_t ctx = {0};
  Rectangle_center_TypeDef ref_roi;
  int32_t ret;

  app_split2_ensure_init(&ctx, tracker);

  /* Default local-search reference ROI:
   * when lost, freeze the search center at the last confirmed good position */
  ref_roi = tracker->lost_status ? tracker->last_good_roi : tracker->target_roi;

  /* Update post proc parameters from ref_roi, which is the ROI the search crop is built on.
   * This must stay consistent with app_split2_preproc(): the post-processing decodes the
   * predicted position as an offset relative to target_pos, so target_pos has to be the
   * center of the crop actually fed to the network, not the last reported target position */
  app_update_pp_geometry(&ctx.pp_params, &ref_roi);

  /* Run pre proc (crop and resize the search zone into the NN input buffer) */
  app_split2_preproc(&ctx, capture_buffer, &ref_roi, preproc_ms);

  /* Release the capture buffer which is not needed anymore */
  bqueue_put_free(&nn_input_queue);

  /* Run inference */
  app_split2_infer(&ctx, inf_ms);

  /* Run post-processing */
  ret = app_postprocess_run((void *[]){ctx.nn_out[0], ctx.nn_out[1]}, STAI_VOT_SPLIT2_OUT_NUM,
                            &ctx.pp_output, &ctx.pp_params);
  assert(ret == AI_VOT_LIGHTTRACKM_PP_ERROR_NO);

  /* Run tracker update : handle lost/recovery and update the target ROI */
  app_split2_update_tracker(tracker, &ctx.pp_output);

  tracker->detection_psr = ctx.pp_output.psr;
  tracker->tracking_active = true;
}

static void nn_thread_init_state(tracker_state_t *tracker_state, touch_state_t *touch_state)
{
  uint8_t *nn_pipe_dst;
  int32_t ret;

  assert(tracker_state != NULL);
  assert(touch_state != NULL);

  /* Initialize runtime */
  ret = stai_runtime_init();
  assert(ret == STAI_SUCCESS);

  /* Initialize the tracking state: no target yet, split2 network initialized on first use */
  *tracker_state = (tracker_state_t){0};
  tracker_state->split2_reinit_pending = true;

  /* Initialize the touch state: no selection in progress */
  *touch_state = (touch_state_t){0};

  nn_pipe_dst = bqueue_get_free(&nn_input_queue, 0);
  assert(nn_pipe_dst);
  CAM_NNPipe_Start(nn_pipe_dst, CMW_MODE_CONTINUOUS);
}

static bool nn_thread_handle_selection_phase(tracker_state_t *tracker_state, touch_state_t *touch_state,
                                             uint8_t *capture_buffer, bool *template_ready)
{
  bool selection_ready, touch_valid;
  Rectangle_TypeDef selected_roi;
  TS_State_t ts_state;

  assert(tracker_state != NULL);
  assert(touch_state != NULL);
  assert(capture_buffer != NULL);
  assert(template_ready != NULL);

  /* Single touchscreen read for the whole frame */
  touch_valid = app_touch_read(&ts_state);

  selection_ready = touch_valid && (app_touch_update_selection(touch_state, &ts_state, &selected_roi) != 0);
  if (selection_ready)
  {
    /* Selection valid : run the template NN to get the initial template */
    tracker_state->split2_reinit_pending = true;
    /* app_nn_template_process() releases capture_buffer */
    app_nn_template_process(tracker_state, capture_buffer, &selected_roi);
    tracker_state->lost_status = false;
    *template_ready = true;
  }
  else
  {
    /* No action required, just free the buffer */
    bqueue_put_free(&nn_input_queue);
  }

  return false;
}

static void nn_thread_reset_tracking_on_touch(tracker_state_t *tracker_state, touch_state_t *touch_state,
                                              const TS_State_t *ts_state, bool *template_ready)
{
  assert(tracker_state != NULL);
  assert(touch_state != NULL);
  assert(ts_state != NULL);
  assert(template_ready != NULL);

  /* Touch during tracking: restart the workflow from initial ROI selection */
  *template_ready = false;
  tracker_state->tracking_active = false;
  touch_state->is_dragging = false;
  tracker_state->split2_reinit_pending = true;
  app_touch_clear_preview(touch_state);
  tracker_state->target_roi = (Rectangle_center_TypeDef){0};
  tracker_state->last_good_roi = (Rectangle_center_TypeDef){0};
  tracker_state->lost_frame_count = 0;
  tracker_state->lost_status = false;
  tracker_state->display_roi_valid = false;

  if (ts_state->TouchDetected != 0)
  {
    /* Start new selection */
    app_touch_start_selection_from_point(touch_state, ts_state->TouchX, ts_state->TouchY);
  }
}

static bool nn_thread_handle_tracking_phase(tracker_state_t *tracker_state, touch_state_t *touch_state,
                                            uint8_t *capture_buffer, bool *template_ready,
                                            uint32_t *inf_ms, uint32_t *preproc_ms)
{
  TS_State_t ts_state;
  bool touch_pressed;

  assert(tracker_state != NULL);
  assert(touch_state != NULL);
  assert(capture_buffer != NULL);
  assert(template_ready != NULL);
  assert(inf_ms != NULL);
  assert(preproc_ms != NULL);

  /* Single touchscreen read for the whole frame */
  touch_pressed = app_touch_read(&ts_state) && (ts_state.TouchDetected != 0);

  if (touch_pressed)
  {
    nn_thread_reset_tracking_on_touch(tracker_state, touch_state, &ts_state, template_ready);
    bqueue_put_free(&nn_input_queue);
    /* Skip split2 on this frame because tracking was just reset */
    return true;
  }

  /* Main NN operation: process split2 NN
   * app_nn_split2_process() releases capture_buffer */
  app_nn_split2_process(tracker_state, capture_buffer, inf_ms, preproc_ms);

  return false;
}

static void nn_thread_sync_display(const tracker_state_t *tracker_state, const touch_state_t *touch_state,
                                   uint32_t nn_period_ms, uint32_t inf_ms, uint32_t preproc_ms)
{
  int32_t ret;

  assert(tracker_state != NULL);
  assert(touch_state != NULL);

  ret = xSemaphoreTake(disp.lock, portMAX_DELAY);
  assert(ret == pdTRUE);
  disp.info.inf_ms = inf_ms;
  disp.info.preproc_ms = preproc_ms;
  disp.info.nn_period_ms = nn_period_ms;
  disp.info.detection_psr = tracker_state->detection_psr;
  disp.info.tracking_active = tracker_state->tracking_active;
  disp.info.lost_status = tracker_state->lost_status;
  disp.info.display_roi_valid = tracker_state->display_roi_valid;
  disp.info.display_roi = tracker_state->display_roi;
  disp.info.last_good_roi = tracker_state->last_good_roi;
  disp.info.preview_valid = touch_state->preview_valid;
  disp.info.preview_roi_lcd = touch_state->preview_roi_lcd;
  ret = xSemaphoreGive(disp.lock);
  assert(ret == pdTRUE);

  /* It's possible xqueue is empty if display is slow. So don't check error code that may be pdFALSE in that case */
  xSemaphoreGive(disp.update);
}

static void nn_thread_fct(void *arg)
{
  uint32_t nn_period[2], nn_period_ms, inf_ms = 0, preproc_ms = 0;
  tracker_state_t tracker_state;
  touch_state_t touch_state;
  bool template_ready = false;
  uint8_t *capture_buffer;

  (void)arg;

  nn_thread_init_state(&tracker_state, &touch_state);

  /*** App Loop ***************************************************************/
  nn_period[1] = HAL_GetTick();

  while (1)
  {
    nn_period[0] = nn_period[1];
    nn_period[1] = HAL_GetTick();
    nn_period_ms = nn_period[1] - nn_period[0];

    /* Get a camera buffer */
    capture_buffer = bqueue_get_ready(&nn_input_queue);
    assert(capture_buffer);

    if (!template_ready)
    {
      /* Initial state: waiting for the user to draw a valid selection */
      if (nn_thread_handle_selection_phase(&tracker_state, &touch_state, capture_buffer, &template_ready))
      {
        continue;
      }
    }
    else
    {
      /* Track the object that the user has already selected */
      if (nn_thread_handle_tracking_phase(&tracker_state, &touch_state, capture_buffer, &template_ready,
                                          &inf_ms, &preproc_ms))
      {
        continue;
      }
    }

    nn_thread_sync_display(&tracker_state, &touch_state, nn_period_ms, inf_ms, preproc_ms);
  }
}

static void dp_thread_fct(void *arg)
{
  uint32_t ts, disp_ms = 0;
  display_info_t info;
  HAL_StatusTypeDef hal_ret;
  int32_t ret;

  (void)arg;

  while (1)
  {
    ret = xSemaphoreTake(disp.update, portMAX_DELAY);
    assert(ret == pdTRUE);

    ret = xSemaphoreTake(disp.lock, portMAX_DELAY);
    assert(ret == pdTRUE);
    info = disp.info;
    ret = xSemaphoreGive(disp.lock);
    assert(ret == pdTRUE);
    info.disp_ms = disp_ms;

    ts = HAL_GetTick();

    /* Retarget the BSP drawing helpers to the back buffer */
    hlcd_ltdc.LayerCfg[LTDC_LAYER_2].FBStartAdress = (uint32_t)lcd_fg_buffer[lcd_fg_buffer_rd_idx];

    /* Update display information (boxes, metrics, info) */
    dp_update_network_output(&info);
    CACHE_OP(SCB_CleanDCache_by_Addr(lcd_fg_buffer[lcd_fg_buffer_rd_idx], LCD_FG_WIDTH * LCD_FG_HEIGHT* 2));

    /* Program the new address and request the reload back to back */
    __disable_irq();
    hal_ret = HAL_LTDC_SetAddress_NoReload(&hlcd_ltdc, (uint32_t)lcd_fg_buffer[lcd_fg_buffer_rd_idx], LTDC_LAYER_2);
    assert(hal_ret == HAL_OK);
    hal_ret = HAL_LTDC_ReloadLayer(&hlcd_ltdc, LTDC_RELOAD_VERTICAL_BLANKING, LTDC_LAYER_2);
    assert(hal_ret == HAL_OK);
    __enable_irq();

    lcd_fg_buffer_rd_idx = 1 - lcd_fg_buffer_rd_idx;

    disp_ms = HAL_GetTick() - ts;
  }
}

static void isp_thread_fct(void *arg)
{
  int32_t ret;

  (void)arg;

  while (1)
  {
    ret = xSemaphoreTake(isp_sem, portMAX_DELAY);
    assert(ret == pdTRUE);

    CAM_IspUpdate();
  }
}

static void Display_init(void)
{
  BSP_LCD_LayerConfig_t layer_config = {0};
  int32_t ret;

  ret = BSP_LCD_Init(0, LCD_ORIENTATION_LANDSCAPE);
  assert(ret == BSP_ERROR_NONE);

  layer_config.X0 = lcd_bg_area.x0;
  layer_config.Y0 = lcd_bg_area.y0;
  layer_config.X1 = layer_config.X0 + lcd_bg_area.width;
  layer_config.Y1 = layer_config.Y0 + lcd_bg_area.height;
  layer_config.PixelFormat = LCD_PIXEL_FORMAT_RGB565;
  layer_config.Address = (uint32_t)lcd_bg_buffer[lcd_bg_buffer_disp_idx];
  ret = BSP_LCD_ConfigLayer(0, LTDC_LAYER_1, &layer_config);
  assert(ret == BSP_ERROR_NONE);

  layer_config.X0 = lcd_fg_area.x0;
  layer_config.Y0 = lcd_fg_area.y0;
  layer_config.X1 = layer_config.X0 + lcd_fg_area.width;
  layer_config.Y1 = layer_config.Y0 + lcd_fg_area.height;
  layer_config.PixelFormat = LCD_PIXEL_FORMAT_ARGB4444;
  layer_config.Address = (uint32_t)lcd_fg_buffer[1];
  ret = BSP_LCD_ConfigLayer(0, LTDC_LAYER_2, &layer_config);
  assert(ret == BSP_ERROR_NONE);

  UTIL_LCD_SetFuncDriver(&LCD_Driver);
  UTIL_LCD_SetLayer(LTDC_LAYER_2);

  UTIL_LCD_Clear(UTIL_LCD_COLOR_TRANSPARENT);
  UTIL_LCD_SetFont(&LCD_FONT);
  UTIL_LCD_SetTextColor(TEXT_COLOR);
}

static void app_display_info_header(void)
{
  printf("========================================\n");
  printf("x-cube-n6-ai-visual-object-tracking %s\n", APP_VERSION_STRING);
  printf("Build date & time: %s %s\n", __DATE__, __TIME__);
#if defined(__GNUC__)
  printf("Compiler: GCC %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
#elif defined(__ICCARM__)
  printf("Compiler: IAR EWARM %d.%d.%d\n", __VER__ / 1000000, (__VER__ / 1000) % 1000 ,__VER__ % 1000);
#else
  printf("Compiler: Unknown\n");
#endif
  printf("HAL: %lu.%lu.%lu\n", __STM32N6xx_HAL_VERSION_MAIN, __STM32N6xx_HAL_VERSION_SUB1, __STM32N6xx_HAL_VERSION_SUB2);
  printf("STEdgeAI Tools: %d.%d.%d\n", STAI_TOOLS_VERSION_MAJOR, STAI_TOOLS_VERSION_MINOR, STAI_TOOLS_VERSION_MICRO);
  printf("NN models: %s , %s\n", STAI_VOT_SPLIT2_ORIGIN_MODEL_NAME, STAI_VOT_TEMPLATE_ORIGIN_MODEL_NAME);
  printf("========================================\n");
}

void app_run(void)
{
  UBaseType_t isp_priority = FREERTOS_PRIORITY(2);
  UBaseType_t dp_priority = FREERTOS_PRIORITY(-2);
  UBaseType_t nn_priority = FREERTOS_PRIORITY(1);
  TS_Init_t touch_init;
  TaskHandle_t hdl;
  int32_t ret;

  app_display_info_header();
  /* Enable DWT so DWT_CYCCNT works when debugger not attached */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* GPU2D used for the neural network input crop and resize */
  app_gpu_init();

  /* Screen init */
  memset(lcd_bg_buffer, 0, sizeof(lcd_bg_buffer));
  CACHE_OP(SCB_CleanInvalidateDCache_by_Addr(lcd_bg_buffer, sizeof(lcd_bg_buffer)));
  memset(lcd_fg_buffer, 0, sizeof(lcd_fg_buffer));
  CACHE_OP(SCB_CleanInvalidateDCache_by_Addr(lcd_fg_buffer, sizeof(lcd_fg_buffer)));
  Display_init();

  /* Create buffer queues */
  ret = bqueue_init(&nn_input_queue, 2, (uint8_t *[2]){nn_input_buffers[0], nn_input_buffers[1]});
  assert(ret == 0);

  /* Camera init */
  CAM_Init();

  /* Sem + mutex init */
  isp_sem = xSemaphoreCreateCountingStatic(1, 0, &isp_sem_buffer);
  assert(isp_sem);
  disp.update = xSemaphoreCreateCountingStatic(1, 0, &disp.update_buffer);
  assert(disp.update);
  disp.lock = xSemaphoreCreateMutexStatic(&disp.lock_buffer);
  assert(disp.lock);

  /* Touchscreen init */
  touch_init.Width = LCD_FG_WIDTH;
  touch_init.Height = LCD_FG_HEIGHT;
  touch_init.Orientation = TS_SWAP_NONE;
  touch_init.Accuracy = 8;

  ret = BSP_TS_Init(0, &touch_init);
  assert(ret == BSP_ERROR_NONE);

  app_touch_enable_it();

  /* Start LCD Display camera pipe stream */
  CAM_DisplayPipe_Start(lcd_bg_buffer[0], CMW_MODE_CONTINUOUS);

  /* Threads init */
  hdl = xTaskCreateStatic(nn_thread_fct, "nn", configMINIMAL_STACK_SIZE * 2, NULL, nn_priority, nn_thread_stack, &nn_thread);
  assert(hdl != NULL);
  hdl = xTaskCreateStatic(dp_thread_fct, "dp", configMINIMAL_STACK_SIZE * 2, NULL, dp_priority, dp_thread_stack, &dp_thread);
  assert(hdl != NULL);
  hdl = xTaskCreateStatic(isp_thread_fct, "isp", configMINIMAL_STACK_SIZE * 2, NULL, isp_priority, isp_thread_stack, &isp_thread);
  assert(hdl != NULL);

  printf("Start Processing\n");
}

int CMW_CAMERA_PIPE_FrameEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1)
  {
    app_main_pipe_frame_event();
  }
  else if (pipe == DCMIPP_PIPE2)
  {
    app_ancillary_pipe_frame_event();
  }

  return HAL_OK;
}

int CMW_CAMERA_PIPE_VsyncEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1)
  {
    app_main_pipe_vsync_event();
  }

  return HAL_OK;
}
