/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "driver/chip/hal_flash.h"
#include "driver/chip/hal_wdg.h"
#include "kernel/os/os_mutex.h"
#include "kernel/os/os_time.h"
#include "compiler.h"
#include "image/flash.h"

#include "image_debug.h"

#define FLASH_OPEN_TIMEOUT	(5000)

typedef struct flash_erase_param {
	int32_t         block_size;
	FlashEraseMode	erase_mode;
} flash_erase_param_t;

static const flash_erase_param_t s_flash_erase_param[] = {
	{ (64 * 1024), FLASH_ERASE_64KB },
	{ (32 * 1024), FLASH_ERASE_32KB },
	{ ( 4 * 1024), FLASH_ERASE_4KB  },
};

#define FLASH_ERASE_PARAM_CNT \
	(sizeof(s_flash_erase_param) / sizeof(s_flash_erase_param[0]))

static OS_Mutex_t s_flash_mutex;
static int s_flash_mutex_created;

static int flash_mutex_lock(void)
{
	if (!s_flash_mutex_created) {
		if (OS_MutexCreate(&s_flash_mutex) != OS_OK) {
			FLASH_ERR("flash mutex create failed\n");
			return -1;
		}
		s_flash_mutex_created = 1;
	}

	if (OS_MutexLock(&s_flash_mutex, OS_WAIT_FOREVER) != OS_OK) {
		FLASH_ERR("flash mutex lock failed\n");
		return -1;
	}

	return 0;
}

static void flash_mutex_unlock(void)
{
	if (s_flash_mutex_created) {
		OS_MutexUnlock(&s_flash_mutex);
	}
}

static HAL_Status __nonxip_text flash_erase_blocks(uint32_t flash, FlashEraseMode erase_mode,
                                      uint32_t addr, uint32_t block_size,
                                      uint32_t block_cnt)
{
	uint32_t i;
	HAL_Status status = HAL_OK;

	for (i = 0; i < block_cnt; ++i) {
		uint32_t erase_addr = addr + (i * block_size);

		HAL_WDG_Feed();
		status = HAL_Flash_Erase_BlockingPoll(flash, erase_mode, erase_addr, 1);

		HAL_WDG_Feed();
		if (status != HAL_OK) {
			FLASH_ERR("erase fail, mode:%#x, addr:0x%x(%dK), block:%u/%u\n",
			          erase_mode, erase_addr, erase_addr / 1024, i + 1, block_cnt);
			return status;
		}

		/*
		 * The erase itself runs with XIP/IRQs constrained in the HAL wrapper.
		 * Once that sector has completed and flash is usable again, yield briefly
		 * so the WLAN/MBOX and TCP/IP tasks can drain work during a long OTA erase.
		 */
		OS_MSleep(1);
	}

	return status;
}

/**
 * @brief Read/write an amount of data from/to flash
 * @param[in] flash Flash device number
 * @param[in/out] addr flash address to be read/written
 * @param[in] buf Pointer to the data buffer
 * @param[in] size Number of bytes to be read/written
 * @param[in] do_write Read or write
 * @return Number of bytes read/written
 */
uint32_t flash_rw(uint32_t flash, uint32_t addr,
                  void *buf, uint32_t size, int do_write)
{
	HAL_Status status;

	if (flash_mutex_lock() != 0) {
		return 0;
	}

	status = HAL_Flash_Open(flash, FLASH_OPEN_TIMEOUT);
	if (status != HAL_OK) {
		FLASH_ERR("open %u fail\n", flash);
		flash_mutex_unlock();
		return 0;
	}

	if (do_write) {
		status = HAL_Flash_Write(flash, addr, buf, size);
	} else {
		status = HAL_Flash_Read(flash, addr, buf, size);
	}

	HAL_Flash_Close(flash);
	flash_mutex_unlock();
	if (status != HAL_OK) {
		FLASH_ERR("%s fail, (%u, %#x, %u), %p\n", do_write ? "write" : "read",
				  flash, addr, size, buf);
		return 0;
	}
	return size;
}

/**
 * @brief Get erase block size for the specified area
 * @param[in] flash Flash device number
 * @param[in] addr Start address of the specified area
 * @param[in] size Size of the specified area
 * @return Erase block size, -1 on misaligned
 */
int32_t flash_get_erase_block(uint32_t flash, uint32_t addr, uint32_t size)
{
	int i;
	int32_t block_size;
	FlashEraseMode erase_mode;
	uint32_t start;

	for (i = 0; i < FLASH_ERASE_PARAM_CNT; ++i) {
		block_size = s_flash_erase_param[i].block_size;
		erase_mode = s_flash_erase_param[i].erase_mode;
		if ((size >= block_size) &&
		    ((size & (block_size - 1)) == 0) &&
		    (HAL_Flash_MemoryOf(flash, erase_mode, addr, &start) == HAL_OK) &&
		    (addr == start)) {
		    break;
		}
	}

	if (i >= FLASH_ERASE_PARAM_CNT) {
		FLASH_ERR("(%u, %#x, %u) misaligned\n", flash, addr, size);
		return -1;
	}
	return block_size;
}

/**
 * @brief Erase a specified area in flash
 * @param[in] flash Flash device number
 * @param[in] addr Start address of the specified area
 * @param[in] size Size of the specified area
 * @return 0 on success, -1 on failure
 */
int flash_erase(uint32_t flash, uint32_t addr, uint32_t size)
{
	int i;
	int32_t block_size;
	FlashEraseMode erase_mode;
	uint32_t start;
	HAL_Status status;

	FLASH_DBG("flash_erase called for %i size %i\n", addr, size);

	if (flash_mutex_lock() != 0) {
		return -1;
	}

	if (HAL_Flash_Open(flash, FLASH_OPEN_TIMEOUT) != HAL_OK) {
		FLASH_ERR("open %d fail\n", flash);
		flash_mutex_unlock();
		return -1;
	}

	for (i = 0; i < FLASH_ERASE_PARAM_CNT; ++i) {
		block_size = s_flash_erase_param[i].block_size;
		erase_mode = s_flash_erase_param[i].erase_mode;
		if ((size >= block_size) &&
		    ((size & (block_size - 1)) == 0) &&
		    (HAL_Flash_MemoryOf(flash, erase_mode, addr, &start) == HAL_OK) &&
		    (addr == start)) {
			FLASH_DBG("%s() (%u, %#x, %u), block_size %d, erase_mode %#x\n",
			          __func__, flash, addr, size, block_size, erase_mode);
			status = HAL_Flash_Erase(flash, erase_mode, addr, size / block_size);
			if (status == HAL_OK) {
				FLASH_DBG("%s() success\n", __func__);
				break;
			} else {
				FLASH_WRN("%s() fail, (%u, %#x, %u), block_size %d, "
				          "erase_mode %#x\n", __func__, flash, addr, size,
				          block_size, erase_mode);
			}
		}
	}

	HAL_Flash_Close(flash);
	flash_mutex_unlock();

	if (i >= FLASH_ERASE_PARAM_CNT) {
		FLASH_ERR("fail, (%u, %#x, %u)\n", flash, addr, size);
		return -1;
	}
	return 0;
}

int __nonxip_text flash_erase_wrap(uint32_t flash, uint32_t addr, uint32_t size)
{
	uint32_t end = addr + size;
	uint32_t cur = addr;
	HAL_Status status;
	int ret = 0;

	if (size == 0) {
		return 0;
	}

	if (end < addr) {
		FLASH_ERR("invalid erase range, addr:0x%x, size:0x%x\n", addr, size);
		return -1;
	}

	if ((addr & (s_flash_erase_param[2].block_size - 1)) ||
		(size & (s_flash_erase_param[2].block_size - 1))) {
		FLASH_ERR("erase range is not 4k aligned, addr:0x%x, size:0x%x\n", addr, size);
		return -1;
	}

	if (flash_mutex_lock() != 0) {
		return -1;
	}

	if (HAL_Flash_Open(flash, FLASH_OPEN_TIMEOUT) != HAL_OK) {
		FLASH_ERR("open %d fail\n", flash);
		flash_mutex_unlock();
		return -1;
	}

	FLASH_DBG("erase range start:0x%x(%dK) end:0x%x(%dK) size:0x%x(%dK)\n",
			  addr, addr / 1024, end, end / 1024, size, size / 1024);

	while (cur < end) {
		uint32_t remaining = end - cur;
		uint32_t block_size = s_flash_erase_param[2].block_size;
		FlashEraseMode erase_mode = s_flash_erase_param[2].erase_mode;

		/*
		 * XR809 OTA/runtime erase must keep each XIP-suspended critical section
		 * short.  64K/32K erases can let the WLAN/MBOX side fault while the
		 * flash is unavailable.  Use 4K sectors only for this wrapper path.
		 * The ordinary flash_erase() path remains unchanged for LFS/config users.
		 */
		if (((cur - addr) & 0xFFFF) == 0) {
			FLASH_DBG("erase progress addr:0x%x(%dK), remaining:0x%x(%dK)\n",
			          cur, cur / 1024, remaining, remaining / 1024);
		}

		status = flash_erase_blocks(flash, erase_mode, cur, block_size, 1);
		if (status != HAL_OK) {
			FLASH_ERR("earse fail\n");
			ret = -1;
			goto out;
		}

		cur += block_size;
	}

out:
	HAL_Flash_Close(flash);
	flash_mutex_unlock();

	return ret;
}
