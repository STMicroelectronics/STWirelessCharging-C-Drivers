/**
  ******************************************************************************
  * @file    stwlc98.c
  * @brief   This file provides set of firmware functions to manage:
  *          - STWLC98 wireless charger from STMicroelectronics
  ******************************************************************************
  * @attention
  *
  * This file is part of stwlc98-pid.
  *
  * Copyright (c) 2023, STMicroelectronics - All Rights Reserved
  * Author(s): ACD (Analog Custom Devices) Software Team for STMicroelectronics.
  *
  * License terms: BSD 3-clause "New" or "Revised" License.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice, this
  * list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  *
  * 3. Neither the name of the copyright holder nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "stwlc98.h"
#include "nvm_data.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/**
  * @defgroup    STWLC98
  * @brief       This file provides a set of functions needed to drive the
  *              stwlc98 wireless charger.
  * @{
  *
  */

/* Common_Definitions */
#define STWLC98_MAX_READ_CHUNK				                        500U
#define STWLC98_MAX_WRITE_CHUNK					                    250U
#define STWLC98_LOG_SIZE						                    1024U
#define STWLC98_GENERAL_POLLING_MS				                    50U
#define STWLC98_GENERAL_TIMEOUT					                    100U
#define STWLC98_RESET_DELAY_MS					                    50U
#define STWLC98_I2C_CHUNK_SIZE					                    128
#define STWLC98_AFTER_SYS_RESET_SLEEP_MS		                    50
#define STWLC98_WRITE_READ_OPERATION			                    0x01
#define STWLC98_WRITE_OPERATION					                    0x02
#define STWLC98_MIN_WR_BYTE_LENGTH			    	                5
#define STWLC98_MIN_W_BYTE_LENGTH				                    4
#define STWLC98_MAX_RETRY						                    3
#define STWLC98_OPCODE_WRITE					                    0xFA
#define STWLC98_OPCODE_SIZE						                    1U

/* NVM Definitions */
#define STWLC98_NVM_FULL_ERASE_INTERVAL_MS		                    10U
#define STWLC98_NVM_FULL_ERASE_TIMEOUT			                    100U
#define STWLC98_NVM_ERASE_INTERVAL_MS			                    1U
#define STWLC98_NVM_ERASE_TIMEOUT				                    20U
#define STWLC98_NVM_WRITE_INTERVAL_MS			                    1U
#define STWLC98_NVM_WRITE_TIMEOUT				                    20U
#define STWLC98_NVM_UPDATE_MAX_RETRY			                    3U
#define STWLC98_NVM_SECTOR_BYTE_SIZE			                    128U
#define STWLC98_NVM_PATCH_START_SECTOR_INDEX	                    0
#define STWLC98_NVM_CFG_START_SECTOR_INDEX		                    125
#define STWLC98_NVM_PATCH_TOTAL_SECTORS			                    125
#define STWLC98_NVM_CFG_TOTAL_SECTORS			                    3U
#define STWLC98_NVM_PATCH_SIZE					                    (STWLC98_NVM_PATCH_TOTAL_SECTORS * STWLC98_NVM_SECTOR_BYTE_SIZE)
#define STWLC98_NVM_CFG_SIZE			                            (STWLC98_NVM_CFG_TOTAL_SECTORS * STWLC98_NVM_SECTOR_BYTE_SIZE)
#define STWLC98_NVM_ERASE_ALL_SLEEP_MS		                        10U
#define STWLC98_NVM_ERASE_ALL_POLLING_TIMEOUT			            100U
#define STWLC98_NVM_SECTOR_OPERATION_SLEEP_MS			            1U
#define STWLC98_NVM_SECTOR_OPERATION_POLLING_TIMEOUT				20U
#define STWLC98_NVM_CFG_ADDR						                (STWLC98_NVM_PATCH_ADDR +		\
	                                                                (STWLC98_NVM_PATCH_TOTAL_SECTORS * STWLC98_NVM_SECTOR_BYTE_SIZE))
#define STWLC98_DEFAULT_CFG_VER					                    0x1000
#define STWLC98_DEFAULT_PATCH_VER				                    0x0000
#define STWLC98_DEFAULT_CUST_INFO_VER			                    0x00

/* HW Registers */
#define STWLC98_NVM_PATCH_ADDR					                    0x00080000
#define STWLC98_FDMA_WIN_MODE_ADDR				                    0x00088008
#define STWLC98_TEST_MODE_DUMP_ADDR				                    0x0008800C
#define STWLC98_HWREG_HW_CHIP_ID_ADDR			                    0x2001C000
#define STWLC98_HWREG_HW_VER_ADDR				                    0x2001C002
#define STWLC98_HWREG_RST_ADDR					                    0x2001C138
#define STWLC98_HWREG_CLK_DIV_REG_ADDR			                    0x2001C13A
#define STWLC98_HWREG_ILOAD_ADDR				                    0x2001C150
#define STWLC98_HWREG_NVM_CTRL_ADDR				                    0x2001C1D0
#define STWLC98_HWREG_TEST_MODE_ADDR			                    0x2001C1F0

/* FW Registers */
#define STWLC98_FWREG_CHIP_ID_ADDR				                    0x0000
#define STWLC98_FWREG_OP_MODE_ADDR				                    0x000E
#define STWLC98_FWREG_SYS_CMD_ADDR				                    0x0020
#define STWLC98_FWREG_NVM_WR_PWD_ADDR			                    0x0022
#define STWLC98_FWREG_NVM_SECTOR_INDEX_ADDR		                    0x0024
#define STWLC98_FWREG_SYS_ERR_LATCH_ADDR		                    0x002D
#define STWLC98_FWREG_AUX_DATA_00_ADDR			                    0x0180


enum stwlc98_op_mode {
	OP_MODE_SA = 1,
	OP_MODE_RX = 2,
	OP_MODE_TX = 3,
};

enum stwlc98_level {
	LOG_ERR = 0,
	LOG_INF,
	LOG_DBG,
};


static int32_t stwlc98_read_single_fwreg(struct stwlc98_dev *dev, uint16_t reg, uint8_t *data);
static int32_t stwlc98_write_single_fwreg(struct stwlc98_dev *dev, uint16_t reg, uint8_t data);
static int32_t stwlc98_read_multi_fwreg(struct stwlc98_dev *dev, uint16_t reg, uint8_t *data, int32_t len);
static int32_t stwlc98_write_multi_fwreg(struct stwlc98_dev *dev, uint16_t reg, const uint8_t *data, int32_t len);
static int32_t stwlc98_read_single_hwreg(struct stwlc98_dev *dev, uint32_t reg, uint8_t *data);
static int32_t stwlc98_write_single_hwreg(struct stwlc98_dev *dev, uint32_t reg, uint8_t data);
static int32_t stwlc98_read_multi_hwreg(struct stwlc98_dev *dev, uint32_t reg, uint8_t *data, int32_t len);
static int32_t stwlc98_write_multi_hwreg(struct stwlc98_dev *dev, uint32_t reg, const uint8_t *data, int32_t len);
static void stwlc98_system_reset(struct stwlc98_dev *dev);
static int32_t stwlc98_nvm_write_sector(struct stwlc98_dev *dev, const uint8_t *data, int32_t len, uint8_t sec_idx);
static int32_t stwlc98_nvm_write_bulk(struct stwlc98_dev *dev, const uint8_t *data, int32_t len, uint8_t sec_idx);
static int32_t stwlc98_nvm_erase_bulk(struct stwlc98_dev *dev, uint8_t len, uint8_t sec_idx);
static int32_t stwlc98_nvm_erase_sector(struct stwlc98_dev *dev, uint8_t sec_idx);
static int32_t stwlc98_nvm_full_erase(struct stwlc98_dev *dev);
static int32_t stwlc98_nvm_write(struct stwlc98_dev *dev, enum stwlc98_fw_type fw_type,
		const uint8_t *patch, int32_t patch_size, const uint8_t *cfg, int32_t cfg_size, int32_t erase_only);


/**
  * @defgroup    Utility_Functions
  * @brief       This section provide a set of utility functions used with
  *              the device.
  * @{
  *
  */

/**
  * @brief  Wrapper function to print log message.
  *
  * @param  dev   platform interface definitions(ptr)
  * @param  level log level of the message. 0 -> error, 1 -> info
  * @param  msg   pointer to message to print(ptr)
  * @retval       void
  *
  */
void __weak stwlc98_log(struct stwlc98_dev *dev, int32_t level, const char *msg, ...)
{
	static char log_buff[STWLC98_LOG_SIZE];
	va_list args;

	if (!dev->log || dev->log_info < level)
		return;

	va_start(args, msg);
	switch (level) {
	case LOG_INF:
		sprintf(log_buff, "[ST-INF]: ");
		break;
	default:	/* LOG_ERR */
		sprintf(log_buff, "[ST-ERR]: ");
		break;
	}
	vsprintf(log_buff + 10, msg, args);
	va_end(args);
	dev->log(dev->phandle, level, log_buff, strlen(log_buff));
}

/**
  * @}
  *
  */

/**
  * @defgroup    Register_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a firmware and hardware register of the device.
  * @{
  *
  */

/**
  * @brief  Read device firmware register
  *
  * @param  dev   platform interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       error status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t stwlc98_read_fwreg(struct stwlc98_dev *dev, uint16_t reg, uint8_t *data, int32_t len)
{
  int32_t ret;
  uint8_t *wbuf = NULL;
  int32_t wlen = sizeof(uint16_t);

  wbuf = (uint8_t *)dev->alloc_mem(sizeof(uint8_t) * wlen);
  if (!wbuf) {
	  return STWLC98_ERR_ALLOC_MEM;
  }

  wbuf[0] = (uint8_t)((reg & 0xFF00) >> 8);
  wbuf[1] = (uint8_t)(reg & 0x00FF);

  ret = dev->bus_write_read(dev->phandle, wbuf, wlen, data, len);
  if (ret) {
	  ret = STWLC98_ERR_BUS_WR;
	  stwlc98_log(dev, LOG_DBG, "failed to read register from 0x%04X. ERROR: 0x%08X\r\n", reg, ret);
  }

  dev->free_mem(wbuf);
  return ret;
}

/**
  * @brief  Write device firmware register
  *
  * @param  dev   platform interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       error status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t stwlc98_write_fwreg(struct stwlc98_dev *dev, uint16_t reg, const uint8_t *data, int32_t len)
{
  int32_t ret;
  uint8_t *wbuf = NULL;
  int32_t wlen = sizeof(uint16_t) + len;
  int32_t i;

  wbuf = (uint8_t *)dev->alloc_mem(sizeof(uint8_t) * wlen);
  if (!wbuf) {
	  return STWLC98_ERR_ALLOC_MEM;
  }

  wbuf[0] = (uint8_t)((reg & 0xFF00) >> 8);
  wbuf[1] = (uint8_t)(reg & 0x00FF);
  for (i = 0; i < len; i++)
	  wbuf[2 + i] = data[i];

  ret = dev->bus_write(dev->phandle, wbuf, wlen);
  if (ret) {
	  ret = STWLC98_ERR_BUS_W;
	  stwlc98_log(dev, LOG_DBG, "failed to write register to 0x%04X. ERROR: 0x%08X\r\n", reg, ret);
  }

  dev->free_mem(wbuf);
  return ret;
}

/**
  * @brief  Read single byte from device firmware register
  *
  * @param  dev   platform interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @retval       error status (MANDATORY: return 0 -> no Error)
  *
  */
static int32_t stwlc98_read_single_fwreg(struct stwlc98_dev *dev, uint16_t reg, uint8_t *data)
{
	return stwlc98_read_fwreg(dev, reg, data, 1);
}

/**
  * @brief  Write single byte to device firmware register
  *
  * @param  dev   platform interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  data to write in register reg
  * @retval       error status (MANDATORY: return 0 -> no Error)
  *
  */
static int32_t stwlc98_write_single_fwreg(struct stwlc98_dev *dev, uint16_t reg, uint8_t data)
{
	return stwlc98_write_fwreg(dev, reg, &data, 1);
}

/**
  * @brief  Read multiple bytes from device firmware register
  *
  * @param  dev   platform interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       error status (MANDATORY: return 0 -> no Error)
  *
  */
static int32_t stwlc98_read_multi_fwreg(struct stwlc98_dev *dev, uint16_t reg, uint8_t *data, int32_t len)
{
	return stwlc98_read_fwreg(dev, reg, data, len);
}

/**
  * @brief  Write multiple bytes to device firmware register
  *
  * @param  dev   platform interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       error status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t stwlc98_write_multi_fwreg(struct stwlc98_dev *dev, uint16_t reg, const uint8_t *data, int32_t len)
{
	return stwlc98_write_fwreg(dev, reg, data, len);
}


/**
  * @brief  Read device hardware register
  *
  * @param  dev   platform interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       error status (MANDATORY: return 0 -> no Error)
  *
  */
static int32_t stwlc98_read_hwreg(struct stwlc98_dev *dev, uint32_t reg, uint8_t *data, int32_t len)
{
  int32_t ret;
  uint8_t *wbuf = NULL;
  int32_t wlen = STWLC98_OPCODE_SIZE + sizeof(uint32_t);

  wbuf = (uint8_t *)dev->alloc_mem(sizeof(uint8_t) * wlen);
  if (!wbuf) {
	  return STWLC98_ERR_ALLOC_MEM;
  }

  wbuf[0] = (uint8_t)STWLC98_OPCODE_WRITE;
  wbuf[1] = (uint8_t)((reg & 0xFF000000) >> 24);
  wbuf[2] = (uint8_t)((reg & 0x00FF0000) >> 16);
  wbuf[3] = (uint8_t)((reg & 0x0000FF00) >> 8);
  wbuf[4] = (uint8_t)(reg & 0x000000FF);

  ret = dev->bus_write_read(dev->phandle, wbuf, wlen, data, len);
  if (ret) {
	  ret = STWLC98_ERR_BUS_WR;
	  stwlc98_log(dev, LOG_DBG, "failed to read register from 0x%08X. ERROR: 0x%08X\r\n", reg, ret);
  }

  dev->free_mem(wbuf);
  return ret;
}

/**
  * @brief  Write device hardware register
  *
  * @param  dev   platform interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
static int32_t stwlc98_write_hwreg(struct stwlc98_dev *dev, uint32_t reg, const uint8_t *data, int32_t len)
{
  int32_t ret;
  uint8_t *wbuf = NULL;
  int32_t wlen = STWLC98_OPCODE_SIZE + sizeof(uint32_t) + len;
  int32_t i;

  wbuf = (uint8_t *)dev->alloc_mem(sizeof(uint8_t) * wlen);
  if (!wbuf) {
	  return STWLC98_ERR_ALLOC_MEM;
  }

  wbuf[0] = (uint8_t)STWLC98_OPCODE_WRITE;
  wbuf[1] = (uint8_t)((reg & 0xFF000000) >> 24);
  wbuf[2] = (uint8_t)((reg & 0x00FF0000) >> 16);
  wbuf[3] = (uint8_t)((reg & 0x0000FF00) >> 8);
  wbuf[4] = (uint8_t)(reg & 0x000000FF);
  for (i = 0; i < len; i++)
	  wbuf[5 + i] = data[i];

  ret = dev->bus_write(dev->phandle, wbuf, wlen);
  if (ret) {
	  ret = STWLC98_ERR_BUS_W;
	  stwlc98_log(dev, LOG_DBG, "failed to write register to 0x%08X. ERROR: 0x%08X\r\n", reg, ret);
  }

  dev->free_mem(wbuf);
  return ret;
}


/**
  * @brief  Read single byte from device hardware register
  *
  * @param  dev   platform interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @retval       error status (MANDATORY: return 0 -> no Error)
  *
  */
static int32_t stwlc98_read_single_hwreg(struct stwlc98_dev *dev, uint32_t reg, uint8_t *data)
{
	return stwlc98_read_hwreg(dev, reg, data, 1);
}

/**
  * @brief  Write single byte to device hardware register
  *
  * @param  dev   platform interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  data to write in register reg
  * @retval       error status (MANDATORY: return 0 -> no Error)
  *
  */
static int32_t stwlc98_write_single_hwreg(struct stwlc98_dev *dev, uint32_t reg, uint8_t data)
{
	return stwlc98_write_hwreg(dev, reg, &data, 1);
}

/**
  * @brief  Read multiple bytes from device hardware register
  *
  * @param  dev   platform interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       error status (MANDATORY: return 0 -> no Error)
  *
  */
static int32_t stwlc98_read_multi_hwreg(struct stwlc98_dev *dev, uint32_t reg, uint8_t *data, int32_t len)
{
	int32_t ret;
	int32_t remaining = len;
	int32_t to_read_now = 0;
	int32_t read_already = 0;

	while (remaining > 0)
	{
		to_read_now = remaining > STWLC98_MAX_READ_CHUNK ? STWLC98_MAX_READ_CHUNK : remaining;
		ret = stwlc98_read_hwreg(dev, reg, data + read_already, to_read_now);
		if (ret < 0) return ret;
		remaining -= to_read_now;
		read_already += to_read_now;
		reg += to_read_now;
	}

	return 0;
}

/**
  * @brief  Write multiple bytes to device hardware register
  *
  * @param  dev   platform interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       error status (MANDATORY: return 0 -> no Error)
  *
  */
static int32_t stwlc98_write_multi_hwreg(struct stwlc98_dev *dev, uint32_t reg, const uint8_t *data, int32_t len)
{
	return stwlc98_write_hwreg(dev, reg, data, len);
}

/**
  *@}
  *
  */

/**
  * @defgroup    Common_Functions
  * @brief       This section provide a set of common functions used with device.
  * @{
  *
  */

/**
  * @brief  Read device information
  *
  * @param  dev   platform interface definitions(ptr)
  * @param  info  structure to hold device information(ptr)
  * @retval       error status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t stwlc98_get_chip_info(struct stwlc98_dev *dev, struct stwlc98_chip_info *info)
{
	int32_t ret;
	uint8_t data[14] = { 0 };
    uint32_t sys_err = 0;
    uint8_t cut_id = 0;

	ret = stwlc98_read_multi_fwreg(dev, STWLC98_FWREG_CHIP_ID_ADDR, data, 14);
	ret |= stwlc98_read_multi_fwreg(dev, STWLC98_FWREG_SYS_ERR_LATCH_ADDR, (uint8_t *)&info->sys_err, sizeof(uint32_t));
	ret |= stwlc98_read_single_hwreg(dev, STWLC98_HWREG_HW_VER_ADDR, &info->cut_id);
	if (ret) {
		stwlc98_log(dev, LOG_ERR, "failed to read chip information. ERROR 0x%08X\r\n", ret);
		return ret;
	}

	info->chip_id = (uint16_t)((data[1] << 8) + data[0]);
	info->chip_rev = data[2];
	info->cust_id = data[3];
	info->rom_id = (uint16_t)((data[5] << 8) + data[4]);
	info->patch_id = (uint16_t)((data[7] << 8) + data[6]);
	info->cfg_id = (uint16_t)((data[11] << 8) + data[10]);
	info->pe_id = data[12];
	info->pe_adc_id = data[13];
    info->sys_err = sys_err;
    info->cut_id = cut_id;

	stwlc98_log(dev, LOG_INF, "chip_id: %d chip_rev: %d cust_id: %d rom_id: 0x%04X patch_id: 0x%04X cfg_id: 0x%04X"
			" pe_id: 0x%02X pe_adc_id: 0x%02X sys_err: 0x%08X cut_id: %d\r\n", info->chip_id, info->chip_rev,
			info->cust_id, info->rom_id, info->patch_id, info->cfg_id, info->pe_id, info->pe_adc_id, info->sys_err,
			info->cut_id);

	return 0;
}

/**
  * @brief  Device system reset
  *
  * @param  dev   platform interface definitions(ptr)
  * @retval       error status (MANDATORY: return 0 -> no Error)
  *
  */

static void stwlc98_system_reset(struct stwlc98_dev *dev)
{
    (void)stwlc98_write_single_hwreg(dev, STWLC98_HWREG_RST_ADDR, 0x01);
    dev->mdelay(STWLC98_RESET_DELAY_MS);
}


/**
  * @brief  Write data into specific NVM sector
  *
  * @param  dev       platform interface definitions(ptr)
  * @param  data      pointer to data to write(ptr)
  * @param  len       number of bytes to write
  * @param  sec_idx sector index to write
  * @retval           error status (MANDATORY: return 0 -> no Error)
  *
  */

static int32_t stwlc98_nvm_write_sector(struct stwlc98_dev *dev, const uint8_t *data, int32_t len, uint8_t sec_idx)
{
    int32_t ret;
    int32_t i;
    int32_t timeout = 1;
    uint8_t reg;

    stwlc98_log(dev, LOG_INF, "writing sector %d\r\n", sec_idx);
    ret = stwlc98_write_single_fwreg(dev, STWLC98_FWREG_NVM_SECTOR_INDEX_ADDR, sec_idx);
    ret |= stwlc98_write_multi_fwreg(dev, STWLC98_FWREG_AUX_DATA_00_ADDR, data, len);
    ret |= stwlc98_write_single_fwreg(dev, STWLC98_FWREG_SYS_CMD_ADDR, 0x04);
    if (ret < 0) return ret;

    for (i = 0; i < STWLC98_NVM_WRITE_TIMEOUT; i++)
    {
        dev->mdelay(STWLC98_NVM_WRITE_INTERVAL_MS);
        ret = stwlc98_read_single_fwreg(dev, STWLC98_FWREG_SYS_CMD_ADDR, &reg);
        if (ret < 0) return ret;
        if ((reg & 0x04) == 0)
        {
            timeout = 0;
            break;
        }
    }

    return timeout == 0 ? ret : STWLC98_ERR_TIMEOUT;
}

/**
  * @brief  Split data based on sector size and write into each sector.
  *
  * @param  dev       platform interface definitions(ptr)
  * @param  data      pointer to data to write(ptr)
  * @param  len       number of bytes to write
  * @param  sec_idx starting sector index to write
  * @retval           error status (MANDATORY: return 0 -> no Error)
  *
  */
static int32_t stwlc98_nvm_write_bulk(struct stwlc98_dev *dev, const uint8_t *data, int32_t len, uint8_t sec_idx)
{
    int32_t ret;
    int32_t remaining = len;
    int32_t to_write_now = 0;
    int32_t written_already = 0;

    while (remaining > 0)
    {
        to_write_now = remaining > STWLC98_NVM_SECTOR_BYTE_SIZE
                        ? STWLC98_NVM_SECTOR_BYTE_SIZE : remaining;
        ret = stwlc98_nvm_write_sector(dev, data + written_already,
                                    to_write_now, sec_idx);
        if (ret < 0) return ret;
        remaining -= to_write_now;
        written_already += to_write_now;
        sec_idx++;
    }

    return 0;
}

/**
  * @brief  Erase specific NVM sector
  *
  * @param  dev       platform interface definitions(ptr)
  * @param  sec_idx sector index to erase
  * @retval           error status (MANDATORY: return 0 -> no Error)
  *
  */
static int32_t stwlc98_nvm_erase_sector(struct stwlc98_dev *dev, uint8_t sec_idx)
{
    int32_t ret;
    int32_t i;
    int32_t timeout = 1;
    uint8_t reg;

    stwlc98_log(dev, LOG_INF, "erasing sector %d\r\n", sec_idx);
    ret = stwlc98_write_single_fwreg(dev, STWLC98_FWREG_NVM_SECTOR_INDEX_ADDR, sec_idx);
    ret |= stwlc98_write_single_fwreg(dev, STWLC98_FWREG_SYS_CMD_ADDR, 0x10);
    if (ret < 0) return ret;

    for (i = 0; i < STWLC98_NVM_ERASE_TIMEOUT; i++)
    {
        dev->mdelay(STWLC98_NVM_ERASE_INTERVAL_MS);
        ret = stwlc98_read_single_fwreg(dev, STWLC98_FWREG_SYS_CMD_ADDR, &reg);
        if (ret < 0) return ret;
        if ((reg & 0x10) == 0)
        {
            timeout = 0;
            break;
        }
    }

    return timeout == 0 ? ret : STWLC98_ERR_TIMEOUT;
}

/**
  * @brief  Erase multiple sectors
  *
  * @param  dev           platform interface definitions(ptr)
  * @param  len           number of consecutive sector to erase
  * @param  sec_idx     starting sector index to erase
  * @retval               error status (MANDATORY: return 0 -> no Error)
  *
  */
static int32_t stwlc98_nvm_erase_bulk(struct stwlc98_dev *dev, uint8_t len, uint8_t sec_idx)
{
    int32_t ret;
    int32_t i;

    for (i = sec_idx; i < sec_idx + len; i++)
    {
        ret = stwlc98_nvm_erase_sector(dev, i);
        if (ret < 0) return ret;
    }

    return 0;
}

/**
  * @brief  Full erase on NVM
  *
  * @param  dev   platform interface definitions(ptr)
  * @retval       error status (MANDATORY: return 0 -> no Error)
  *
  */
static int32_t stwlc98_nvm_full_erase(struct stwlc98_dev *dev)
{
    int32_t ret;
    int32_t i;
    int32_t timeout = 1;
    uint8_t reg;

    ret = stwlc98_write_single_fwreg(dev, STWLC98_FWREG_SYS_CMD_ADDR, 0x20);
    if (ret < 0) return ret;

    for (i = 0; i < STWLC98_NVM_FULL_ERASE_TIMEOUT; i++)
    {
        dev->mdelay(STWLC98_NVM_FULL_ERASE_INTERVAL_MS);
        ret = stwlc98_read_single_fwreg(dev, STWLC98_FWREG_SYS_CMD_ADDR, &reg);
        if (ret < 0) return ret;
        if ((reg & 0x20) == 0)
        {
            timeout = 0;
            break;
        }
    }

    return timeout == 0 ? ret : STWLC98_ERR_TIMEOUT;
}


/**
  * @brief  Write patch and cfg into NVM
  *
  * @param  dev         platform interface definitions(ptr)
  * @param  fw_type     type of firmware data to write
  * @param  patch       pointer to patch data to write(ptr)
  * @param  patch_size  size of patch in bytes to write
  * @param  cfg         pointer to cfg data to write(ptr)
  * @param  cfg_size    size of cfg in bytes to write
  * @param  erase_only  1 if erase NVM only, else 0
  * @retval             error status (MANDATORY: return 0 -> no Error)
  */
static int32_t stwlc98_nvm_write(struct stwlc98_dev *dev, enum stwlc98_fw_type fw_type,
        const uint8_t *patch, int32_t patch_size, const uint8_t *cfg, int32_t cfg_size, int32_t erase_only)
{
    int32_t ret;
    uint8_t reg;

    if (!erase_only)
    {
        if (fw_type == STWLC98_FW_PATCH_CFG || fw_type == STWLC98_FW_PATCH)
        {
            if (patch == NULL || patch_size <= 0 || patch_size > STWLC98_NVM_PATCH_SIZE)
            { 
                ret = STWLC98_ERR_INVALID_PARAM;
                stwlc98_log(dev, LOG_ERR, "invalid patch data. ERROR: 0x%08X\r\n", ret);
                return ret;
            }
        }

        if (fw_type == STWLC98_FW_PATCH_CFG || fw_type == STWLC98_FW_CFG)
        {
            if (cfg == NULL || cfg_size <= 0 || cfg_size > STWLC98_NVM_CFG_SIZE)
            {
                ret = STWLC98_ERR_INVALID_PARAM;
                stwlc98_log(dev, LOG_ERR, "invalid cfg data. ERROR: 0x%08X\r\n", ret);
                return ret;
            }
        }
    }

    /* Check op_mode */ 
    
        ret = stwlc98_read_single_fwreg(dev, STWLC98_FWREG_OP_MODE_ADDR, &reg);
        if (ret < 0) return ret;
        stwlc98_log(dev, LOG_INF, "op_mode: %d\r\n", reg);
        if (reg != OP_MODE_SA){
                    stwlc98_log(dev, LOG_ERR, "no DC power detected, nvm programming aborted. ERROR 0x%08X\r\n", ret);
                    return STWLC98_ERR_UNEXPECTED_OP_MODE;

        }
        
    ret = stwlc98_write_single_fwreg(dev, STWLC98_FWREG_SYS_CMD_ADDR, 0x40);
    dev->mdelay(STWLC98_RESET_DELAY_MS);

    /* Check op_mode */ 

     ret = stwlc98_read_single_fwreg(dev, STWLC98_FWREG_OP_MODE_ADDR, &reg);
        if (ret < 0) return ret;
        stwlc98_log(dev, LOG_INF, "op_mode: %d\r\n", reg);
        if (reg != OP_MODE_SA){
                    stwlc98_log(dev, LOG_ERR, "no DC power detected, nvm programming aborted. ERROR 0x%08X\r\n", ret);
                    return STWLC98_ERR_UNEXPECTED_OP_MODE;

        }
            
    if (!(reg == OP_MODE_SA || reg == OP_MODE_TX))
    {
        ret = STWLC98_ERR_UNEXPECTED_OP_MODE;
        stwlc98_log(dev, LOG_ERR, "no DC power detected, nvm programming aborted. ERROR 0x%08X\r\n", ret);
        return ret;
    }

    /* Disable ILOAD */
    ret = stwlc98_write_single_hwreg(dev, STWLC98_HWREG_ILOAD_ADDR, 0xFE);

    /* Unlock NVM */
    ret = stwlc98_write_single_fwreg(dev, STWLC98_FWREG_NVM_WR_PWD_ADDR, 0xC5);

    switch (fw_type)
    {
    case 	STWLC98_FW_PATCH:
        stwlc98_log(dev, LOG_INF, "erase patch in NVM\r\n");
        ret = stwlc98_nvm_erase_bulk(dev, STWLC98_NVM_PATCH_TOTAL_SECTORS, STWLC98_NVM_PATCH_START_SECTOR_INDEX);
        if (ret < 0)
        {
            stwlc98_log(dev, LOG_ERR, "failed to erase cfg in NVM. ERROR 0x%08X\r\n", ret);
            return ret;
        }
        break;
    case STWLC98_FW_CFG:
        stwlc98_log(dev, LOG_INF, "erase cfg in NVM\r\n");
        ret = stwlc98_nvm_erase_bulk(dev, STWLC98_NVM_CFG_TOTAL_SECTORS, STWLC98_NVM_CFG_START_SECTOR_INDEX );
        if (ret < 0)
        {
            stwlc98_log(dev, LOG_ERR, "failed to erase cfg in NVM. ERROR 0x%08X\r\n", ret);
            return ret;
        }
        break;
    default: /* STWLC98_FW_PATCH_CFG */
        stwlc98_log(dev, LOG_INF, "full erase in NVM\r\n");
        ret = stwlc98_nvm_full_erase(dev);
        if (ret < 0)
        {
            stwlc98_log(dev, LOG_ERR, "failed to full erase NVM. ERROR 0x%08X\r\n", ret);
            return ret;
        }
        break;
    }

    if (erase_only)
        return ret;

    if (fw_type == STWLC98_FW_PATCH_CFG || fw_type == STWLC98_FW_PATCH)
    {
        stwlc98_log(dev, LOG_INF, "writing patch into NVM\r\n");
        ret = stwlc98_nvm_write_bulk(dev, patch, patch_size, STWLC98_NVM_PATCH_START_SECTOR_INDEX);
        if (ret < 0)
        {
            stwlc98_log(dev, LOG_ERR, "failed to write patch into NVM. ERROR 0x%08X\r\n", ret);
            return ret;
        }
    }

    if (fw_type == STWLC98_FW_PATCH_CFG || fw_type == STWLC98_FW_CFG)
    {
        stwlc98_log(dev, LOG_INF, "writing cfg into NVM\r\n");
        ret = stwlc98_nvm_write_bulk(dev, cfg, cfg_size, STWLC98_NVM_CFG_START_SECTOR_INDEX);
        if (ret < 0)
        {
            stwlc98_log(dev, LOG_ERR, "failed to write cfg into NVM. ERROR 0x%08X\r\n", ret);
            return ret;
        }
    }

    stwlc98_system_reset(dev);
    ret = stwlc98_write_single_hwreg(dev, STWLC98_HWREG_ILOAD_ADDR, 0xFE);
    return ret;
}


/**
  * @brief  Validate patch and cfg in NVM.
  *
  * @param  dev          platform interface definitions(ptr)
  * @param  fw_type        type of firmware data to validate
  * @param  patch          pointer to patch data to validate(ptr)
  * @param  patch_size  size of patch in bytes to validate
  * @param  cfg          pointer to cfg data to validate(ptr)
  * @param  cfg_size      size of cfg in bytes to validate
  * @retval               error status (MANDATORY: return 0 -> no Error)
  */
static int32_t stwlc98_nvm_validate(struct stwlc98_dev *dev, enum stwlc98_fw_type fw_type, 
        const uint8_t *patch, int32_t patch_size, const uint8_t *cfg, int32_t cfg_size)
{
    int32_t ret;
    uint8_t test_mode[4] = { 0x07, 0x00, 0x00, 0xB1 };
    uint8_t fdma_mode[4] = { 0x20, 0x00, 0x00, 0x20 };
    uint8_t test_set[4] = { 0x60, 0x00, 0x00, 0x20 };
    uint8_t *patch_now = NULL;
    uint8_t *cfg_now = NULL;

    if (fw_type == STWLC98_FW_PATCH_CFG || fw_type == STWLC98_FW_PATCH)
    {
        if (patch_data == NULL || NVM_PATCH_SIZE <= 0 || NVM_PATCH_SIZE > STWLC98_NVM_PATCH_SIZE)
        {
            ret = STWLC98_ERR_INVALID_PARAM;
            stwlc98_log(dev, LOG_ERR, "invalid patch data. ERROR 0x%08X\r\n", ret);
            return ret;
        }
    }

    if (fw_type == STWLC98_FW_PATCH_CFG || fw_type == STWLC98_FW_CFG)
    {
        if (cfg_data == NULL || NVM_CFG_SIZE <= 0 || NVM_CFG_SIZE > STWLC98_NVM_CFG_SIZE)
        {
            ret = STWLC98_ERR_INVALID_PARAM;
            stwlc98_log(dev, LOG_ERR, "invalid cfg data. ERROR 0x%08X\r\n", ret);
            return ret;
        }
    }

    if (fw_type == STWLC98_FW_PATCH_CFG || fw_type == STWLC98_FW_PATCH)
    {
        patch_now = (uint8_t *)dev->alloc_mem(sizeof(uint8_t) * patch_size);
        if (!patch_now)
            return STWLC98_ERR_ALLOC_MEM;
    }

    if (fw_type == STWLC98_FW_PATCH_CFG || fw_type == STWLC98_FW_CFG)
    {
        cfg_now = (uint8_t *)dev->alloc_mem(sizeof(uint8_t) * cfg_size);
        if (!cfg_now)
        {
            if (patch_now != NULL)
            {
                dev->free_mem(patch_now);
                patch_now = NULL;
            }
            return STWLC98_ERR_ALLOC_MEM;
        }
    }

    /* Hold MCU in reset state */
    ret = stwlc98_write_single_hwreg(dev, STWLC98_HWREG_RST_ADDR, 0x01);
    dev->mdelay(STWLC98_RESET_DELAY_MS);

    ret = stwlc98_write_single_hwreg(dev, STWLC98_HWREG_NVM_CTRL_ADDR, 0xFB);
    ret = stwlc98_write_multi_hwreg(dev, STWLC98_HWREG_TEST_MODE_ADDR, test_mode, 4);

    /* Enter ERVrf mode */
    ret = stwlc98_write_multi_hwreg(dev, STWLC98_FDMA_WIN_MODE_ADDR, fdma_mode, 4);

    /* Compare patch and cfg */
    if (fw_type == STWLC98_FW_PATCH_CFG || fw_type == STWLC98_FW_PATCH)
    {
        ret = stwlc98_read_multi_hwreg(dev, STWLC98_NVM_PATCH_ADDR, patch_now, sizeof(uint8_t) * patch_size);
        if (ret < 0) goto exit_free_mem;
        if (memcmp(patch_now, patch, patch_size) != 0)
        {
            ret = STWLC98_ERR_NVM_DATA_CORRUPTED;
            stwlc98_log(dev, LOG_ERR, "patch data compare failed in ERVrf mode. ERROR 0x%08X\r\n", ret);
            goto exit_free_mem;
        }
    }

    if (fw_type == STWLC98_FW_PATCH_CFG || fw_type == STWLC98_FW_CFG)
    {
        ret = stwlc98_read_multi_hwreg(dev, STWLC98_NVM_CFG_ADDR, cfg_now, sizeof(uint8_t) * cfg_size);
        if (ret < 0) goto exit_free_mem;
        if (memcmp(cfg_now, cfg, cfg_size) != 0)
        {
            ret = STWLC98_ERR_NVM_DATA_CORRUPTED;
            stwlc98_log(dev, LOG_ERR, "cfg data compare failed in ERVrf mode. ERROR 0x%08X\r\n", ret);
            goto exit_free_mem;
        }
    }

    /* Enter PRVrf mode */
    ret = stwlc98_write_multi_hwreg(dev, STWLC98_FDMA_WIN_MODE_ADDR, test_set, 4);

    /* Compare patch and cfg */
    if (fw_type == STWLC98_FW_PATCH_CFG || fw_type == STWLC98_FW_PATCH)
    {
        ret = stwlc98_read_multi_hwreg(dev, STWLC98_NVM_PATCH_ADDR, patch_now, sizeof(uint8_t) * patch_size);
        if (ret < 0) goto exit_free_mem;
        if (memcmp(patch_now, patch, patch_size) != 0)
        {
            ret = STWLC98_ERR_NVM_DATA_CORRUPTED;
            stwlc98_log(dev, LOG_ERR, "patch data compare failed in PRVrf mode. ERROR 0x%08X\r\n", ret);
            goto exit_free_mem;
        }
    }

    if (fw_type == STWLC98_FW_PATCH_CFG || fw_type == STWLC98_FW_CFG)
    {
        ret = stwlc98_read_multi_hwreg(dev, STWLC98_NVM_CFG_ADDR, cfg_now, sizeof(uint8_t) * cfg_size);
        if (ret < 0) goto exit_free_mem;
        if (memcmp(cfg_now, cfg, cfg_size) != 0)
        {
            ret = STWLC98_ERR_NVM_DATA_CORRUPTED;
            stwlc98_log(dev, LOG_ERR, "cfg data compare failed in PRVrf mode. ERROR 0x%08X\r\n", ret);
            goto exit_free_mem;
        }
    }

exit_free_mem:
    if (patch_now != NULL)
    {
        dev->free_mem(patch_now);
        patch_now = NULL;
    }
    if (cfg_now != NULL)
    {
        dev->free_mem(cfg_now);
        cfg_now = NULL;
    }
  stwlc98_log(dev, LOG_ERR, "End of valid before reset\r\n");
    stwlc98_system_reset(dev);
    return ret;
}


/**
  * @brief  Perform FW patch and cfg update on STWLC98. Provide option to
  *         update both patch and cfg or patch only or cfg only.
  *
  * @param  dev             platform interface definitions(ptr)
  * @param  fw_type         type of firmware data to write
  * @param  force_update    Set 0 to no update if patch and cfg is up to date.
  * @retval                 error status (MANDATORY: return 0 -> no Error)
  */
int32_t stwlc98_fw_update(struct stwlc98_dev *dev, enum stwlc98_fw_type fw_type, int32_t force_update)
{
    int32_t ret;
    int32_t retry = 0;
    int32_t success = 0;
    struct stwlc98_chip_info chip = { 0 };

    ret = stwlc98_get_chip_info(dev, &chip);
    if (ret < 0) return ret;

    if (chip.chip_id != NVM_TARGET_CHIP_ID)
    {
        stwlc98_log(dev, LOG_ERR, "invalid chip_id. running|header: [%d|%d]. ERROR 0x%08X\r\n", chip.chip_id, NVM_TARGET_CHIP_ID, ret);
        return STWLC98_ERR_UNEXPECTED_CHIP_ID;
    }

    switch (fw_type)
    {
    case STWLC98_FW_PATCH:
        stwlc98_log(dev, LOG_INF, "patch_id - running|header: [0x%04X|0x%04X]\r\n", chip.patch_id, NVM_PATCH_VERSION_ID);
        if (!force_update && (chip.patch_id == NVM_PATCH_VERSION_ID))
        {
            stwlc98_log(dev, LOG_INF, "NVM programming is not required, patch is up to date\r\n");
            return 0;

        }
        break;
    case STWLC98_FW_CFG:
        stwlc98_log(dev, LOG_INF, "cfg_id - running|header: [0x%04X|0x%04X]\r\n", chip.cfg_id, NVM_CFG_VERSION_ID);
        if (!force_update && (chip.cfg_id == NVM_CFG_VERSION_ID))
        {
            stwlc98_log(dev, LOG_INF, "NVM programming is not required, cfg is up to date\r\n");
            return 0;

        }
        break;
    default: /* STWLC98_FW_PATCH_CFG */
        stwlc98_log(dev, LOG_INF, "patch_id - running|header: [0x%04X|0x%04X]\r\n", chip.patch_id, NVM_PATCH_VERSION_ID);
        stwlc98_log(dev, LOG_INF, "cfg_id - running|header: [0x%04X|0x%04X]\r\n", chip.cfg_id, NVM_CFG_VERSION_ID);
        if (!force_update && (chip.patch_id == NVM_PATCH_VERSION_ID && chip.cfg_id == NVM_CFG_VERSION_ID))
        {
            stwlc98_log(dev, LOG_INF, "NVM programming is not required, both cfg and patch are up to date\r\n");
            return 0;
        }
        break;
    }

    while (retry < STWLC98_NVM_UPDATE_MAX_RETRY)
    {
        ret = stwlc98_nvm_write(dev, fw_type, patch_data, NVM_PATCH_SIZE, cfg_data, NVM_CFG_SIZE, 0);
        if (ret < 0)
            goto exit_system_reset;

        ret = stwlc98_get_chip_info(dev, &chip);
        if (ret < 0) goto exit_system_reset;

        if (fw_type == STWLC98_FW_PATCH_CFG || fw_type == STWLC98_FW_PATCH)
        {
            stwlc98_log(dev, LOG_INF, "patch_id - running|header: [0x%04X|0x%04X]\r\n", chip.patch_id, NVM_PATCH_VERSION_ID);
            if (chip.patch_id != NVM_PATCH_VERSION_ID)
            {
                ret = STWLC98_ERR_NVM_ID_MISMATCH;
                stwlc98_log(dev, LOG_ERR, "patch_id is mismatch after NVM programming. ERROR: 0x%08X\r\n", ret);
                retry++;
                stwlc98_log(dev, LOG_INF, "NVM programming retry: %d\r\n", retry);
                continue;
            }
        }

        if (fw_type == STWLC98_FW_PATCH_CFG || fw_type == STWLC98_FW_CFG)
        {
            stwlc98_log(dev, LOG_INF, "cfg_id - running|header: [0x%04X|0x%04X]\r\n", chip.cfg_id, NVM_CFG_VERSION_ID);
            if (chip.cfg_id != NVM_CFG_VERSION_ID)
            {
                ret = STWLC98_ERR_NVM_ID_MISMATCH;
                stwlc98_log(dev, LOG_ERR, "cfg_id is mismatch after NVM programming. ERROR: 0x%08X\r\n", ret);
                retry++;
                stwlc98_log(dev, LOG_INF, "NVM programming retry: %d\r\n", retry);
                continue;
            }
        }

        ret = stwlc98_nvm_validate(dev, fw_type, patch_data, NVM_PATCH_SIZE, cfg_data, NVM_CFG_SIZE);
        if (ret < 0)
        {
            retry++;
            stwlc98_log(dev, LOG_INF, "NVM programming retry: %d\r\n", retry);
            stwlc98_write_single_hwreg(dev, STWLC98_HWREG_ILOAD_ADDR, 0xFE);
            continue;
        }

        success = 1;
        break;
    }

    if (!success)
    {
        stwlc98_nvm_write(dev, fw_type, patch_data, NVM_PATCH_SIZE, cfg_data, NVM_CFG_SIZE, 1);
        goto exit_system_reset;
    }

    stwlc98_log(dev, LOG_INF, "NVM programming successfully\r\n");
    return 0;

exit_system_reset:
    stwlc98_log(dev, LOG_ERR, "NVM programming failed. ERROR 0x%08X\r\n", ret);
    stwlc98_system_reset(dev);
    return ret;
}


/**
  * @}
  *
  */

