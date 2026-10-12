/**
 ******************************************************************************
 * @file    platform_pool_config.h
 * @author  GPM Application Team
 * @brief   NemaGFX memory pool definition for the TSi allocator
 *
 *          This application only uses the GPU for image crop and resize: no
 *          framebuffer and no vector graphics stencil buffer are allocated
 *          through NemaGFX, so a single pool holding the ring buffer and the
 *          command list is enough
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
#ifndef PLATFORM_POOL_CONFIG_H__
#define PLATFORM_POOL_CONFIG_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint32_t base_addr;
  uint32_t mem_size;
  uint32_t allocated;
} nema_mem_pool_desc_t;

/* Build flags */
#define NEMA_USE_TSI_MALLOC  /* use the TSi memory allocator */
#define NEMA_MULTI_MEM_POOLS /* use memory pools, requires NEMA_USE_TSI_MALLOC */

#define NEMA_MULTI_MEM_POOLS_CNT 1

/* Pool 0 : NemaGFX ring buffer and command lists. Must be reachable by the GPU2D
 * master, so it is placed in AXISRAM (see nema_gfx_mem_pool in nema_hal_freertos.c) */
#define NEMA_MEM_POOL_CL 0
#define GFX_MEM_CL_SIZE (32 * 1024)

extern uint8_t nema_gfx_mem_pool[GFX_MEM_CL_SIZE];

static nema_mem_pool_desc_t memPools[NEMA_MULTI_MEM_POOLS_CNT] =
{
  {(uint32_t)nema_gfx_mem_pool, GFX_MEM_CL_SIZE, 0}, /* Pool 0 */
};

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_POOL_CONFIG_H__ */
